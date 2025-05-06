#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <sys/mman.h>
#include <vector>
#include <memory>
#include <cstdio>
#include <iomanip>
#include <csignal>
#include <sstream>
#include <ctime>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <libcamera/camera_manager.h>
#include <libcamera/camera.h>
#include <libcamera/framebuffer_allocator.h>
#include <libcamera/request.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/control_ids.h>
#include <libcamera/logging.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// MAVLink headers
#include "mavlink/common/mavlink.h"

extern "C" {
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
}

#define RAD_TO_DEG 57.2957795131 // 180/PI for converting radians to degrees

using namespace libcamera;

// Global flag for graceful shutdown
volatile sig_atomic_t running = 1;

// MAVLink configuration
#define SERIAL_PORT "/dev/ttyACM0"  // Pixhawk serial port
#define BAUDRATE B921600            // Typical PX4 baudrate
#define SYSTEM_ID 42                // ID of the PX4 (typically 1)
#define COMPONENT_ID 1              // Component ID of the PX4 autopilot
#define OUR_SYSTEM_ID 255           // ID for our system
#define OUR_COMPONENT_ID 1          // Our component ID

// Structure to hold vehicle pose data
struct VehiclePose {
    Eigen::Vector3d position_ned{0, 0, 0};
    Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
    std::chrono::steady_clock::time_point timestamp;
    bool valid = false;
    std::mutex mutex;
};

// Global variables to store the most recent MAVLink data
struct MAVLinkData {
    float lpos_x, lpos_y, lpos_z;         // LOCAL_POSITION_NED
    float attitude_roll, attitude_pitch, attitude_yaw;  // ATTITUDE
    std::mutex mutex;
    bool valid = false;
} mavlink_data;

// Global vehicle pose data
VehiclePose vehicle_pose;

// Log file streams
std::ofstream position_log_file;
std::ofstream attitude_log_file;
std::ofstream tag_orientation_log_file;
std::chrono::steady_clock::time_point global_start_time;

// Global MAVLink file descriptor for sending messages from other threads
int global_mavlink_fd = -1;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Stopping...\n";
    running = 0;
}

// Format timestamp for logging (MM:SS.xxx format)
std::string format_timestamp(std::chrono::steady_clock::time_point now) {
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - global_start_time);
    int total_ms = elapsed.count();
    int minutes = total_ms / 60000;
    int seconds = (total_ms % 60000) / 1000;
    int milliseconds = total_ms % 1000;
    
    std::stringstream timestamp_stream;
    timestamp_stream << std::setfill('0') << std::setw(2) << minutes << ":"
                     << std::setfill('0') << std::setw(2) << seconds << "."
                     << std::setfill('0') << std::setw(3) << milliseconds;
    return timestamp_stream.str();
}

// Initialize log files with appropriate headers
bool init_log_files(const std::string& timestamp_str) {
    // Create filenames with timestamps
    std::string position_filename = "position_log_" + timestamp_str + ".txt";
    std::string attitude_filename = "attitude_log_" + timestamp_str + ".txt";
    std::string tag_orientation_filename = "tag_orientation_log_" + timestamp_str + ".txt";
    
    // Open position log file
    position_log_file.open(position_filename);
    if (!position_log_file.is_open()) {
        std::cerr << "Error opening position log file: " << position_filename << std::endl;
        return false;
    }
    
    // Open attitude log file
    attitude_log_file.open(attitude_filename);
    if (!attitude_log_file.is_open()) {
        std::cerr << "Error opening attitude log file: " << attitude_filename << std::endl;
        position_log_file.close();
        return false;
    }
    
    // Open tag orientation log file
    tag_orientation_log_file.open(tag_orientation_filename);
    if (!tag_orientation_log_file.is_open()) {
        std::cerr << "Error opening tag orientation log file: " << tag_orientation_filename << std::endl;
        position_log_file.close();
        attitude_log_file.close();
        return false;
    }
    
    // Write log file headers
    position_log_file << "timestamp msgid x y z" << std::endl;
    position_log_file << "MM:SS.xxx ID m m m" << std::endl;
    
    attitude_log_file << "timestamp msgid roll pitch yaw" << std::endl;
    attitude_log_file << "MM:SS.xxx ID deg deg deg" << std::endl;
    
    tag_orientation_log_file << "timestamp tagid tag_x tag_y tag_z tag_roll tag_pitch tag_yaw" << std::endl;
    tag_orientation_log_file << "MM:SS.xxx ID m m m deg deg deg" << std::endl;
    
    std::cout << "Log files initialized:" << std::endl;
    std::cout << "  Position data: " << position_filename << std::endl;
    std::cout << "  Attitude data: " << attitude_filename << std::endl;
    std::cout << "  Tag orientation data: " << tag_orientation_filename << std::endl;
    
    return true;
}

// Close all log files
void close_log_files() {
    if (position_log_file.is_open()) {
        position_log_file.close();
    }
    if (attitude_log_file.is_open()) {
        attitude_log_file.close();
    }
    if (tag_orientation_log_file.is_open()) {
        tag_orientation_log_file.close();
    }
}

// Serial port initialization for MAVLink communication
int serial_init(const char *port, speed_t baudrate) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "Error opening %s: %s\n", port, strerror(errno));
        return -1;
    }

    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baudrate
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    
    // Local mode flags
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Input mode flags
    options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    // Output mode flags
    options.c_oflag &= ~OPOST;
    
    // Control mode flags
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;         // 8 data bits
    options.c_cflag &= ~PARENB;     // No parity
    options.c_cflag &= ~CSTOPB;     // 1 stop bit
    options.c_cflag &= ~CRTSCTS;    // No hardware flow control
    
    // Special characters
    options.c_cc[VMIN] = 0;         // Non-blocking read
    options.c_cc[VTIME] = 1;        // 0.1 seconds read timeout
    
    // Apply settings
    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        fprintf(stderr, "Error configuring serial port: %s\n", strerror(errno));
        close(fd);
        return -1;
    }
    
    return fd;
}

// Function to send a heartbeat message
void send_heartbeat(int fd) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack the message
    mavlink_msg_heartbeat_pack(OUR_SYSTEM_ID, OUR_COMPONENT_ID, &msg,
                              MAV_TYPE_GCS,        // Type of system (Ground Control Station)
                              MAV_AUTOPILOT_INVALID, // Autopilot type (not applicable for GCS)
                              MAV_MODE_FLAG_SAFETY_ARMED, // Mode flags
                              0,                   // Custom mode
                              MAV_STATE_ACTIVE);   // System state
    
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send the message
    ssize_t bytes_sent = write(fd, buf, len);
    if (bytes_sent != len) {
        fprintf(stderr, "Error sending heartbeat: %s\n", strerror(errno));
    }
}

// Function to send a LANDING_TARGET message
void send_landing_target(int fd, uint64_t time_usec, uint8_t target_num, 
                         float x, float y, float z, float q_w, float q_x, float q_y, float q_z) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack the LANDING_TARGET message
    mavlink_msg_landing_target_pack(OUR_SYSTEM_ID, OUR_COMPONENT_ID, &msg,
                                   time_usec,       // Timestamp (microseconds since system boot or UNIX epoch)
                                   target_num,      // The ID of the target if multiple targets are present
                                   MAV_FRAME_LOCAL_NED, // Coordinate frame - using LOCAL_NED
                                   0.0f,            // angle_x (set to 0 as requested)
                                   0.0f,            // angle_y (set to 0 as requested)
                                   0.0f,            // distance (set to 0 as requested)
                                   0.0f,            // size_x (set to 0 as requested)
                                   0.0f,            // size_y (set to 0 as requested)
                                   x,               // x position of the landing target in MAV_FRAME
                                   y,               // y position of the landing target in MAV_FRAME
                                   z,               // z position of the landing target in MAV_FRAME
                                   q_w,             // Quaternion of landing target orientation (w, x, y, z)
                                   q_x,
                                   q_y,
                                   q_z,
                                   LANDING_TARGET_TYPE_VISION_FIDUCIAL, // Type of landing target
                                   1);              // Position valid (boolean: 0 = invalid, 1 = valid)
    
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send the message
    ssize_t bytes_sent = write(fd, buf, len);
    if (bytes_sent != len) {
        fprintf(stderr, "Error sending LANDING_TARGET message: %s\n", strerror(errno));
    } else {
        printf("LANDING_TARGET message sent to PX4\n");
    }
}

// Function to request data stream from flight controller
void request_data_stream(int fd, uint8_t stream_id, uint16_t rate) {
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    // Pack the message
    mavlink_msg_request_data_stream_pack(OUR_SYSTEM_ID, OUR_COMPONENT_ID, &msg,
                                        SYSTEM_ID,    // Target system ID (PX4)
                                        COMPONENT_ID, // Target component ID (PX4 autopilot)
                                        stream_id,    // The ID of the requested data stream
                                        rate,         // Rate in Hz
                                        1);           // Start/stop (1=start, 0=stop)
    
    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    
    // Send the message
    ssize_t bytes_sent = write(fd, buf, len);
    if (bytes_sent != len) {
        fprintf(stderr, "Error requesting data stream: %s\n", strerror(errno));
    } else {
        printf("Data stream request sent for stream %d at %d Hz\n", stream_id, rate);
    }
}

// Log position data from LOCAL_POSITION_NED message
void log_position_data(const mavlink_local_position_ned_t& lpos, uint16_t msgid) {
    if (!position_log_file.is_open()) return;
    
    auto now = std::chrono::steady_clock::now();
    std::string timestamp = format_timestamp(now);
    
    position_log_file << timestamp << " " << msgid << " "
                     << std::fixed << std::setprecision(6)
                     << lpos.x << " " << lpos.y << " " << lpos.z << std::endl;
    
    position_log_file.flush();
}

// Log attitude data from ATTITUDE message
void log_attitude_data(const mavlink_attitude_t& attitude, uint16_t msgid) {
    if (!attitude_log_file.is_open()) return;
    
    auto now = std::chrono::steady_clock::now();
    std::string timestamp = format_timestamp(now);
    
    // Convert radians to degrees
    float roll_deg = attitude.roll * RAD_TO_DEG;
    float pitch_deg = attitude.pitch * RAD_TO_DEG;
    float yaw_deg = attitude.yaw * RAD_TO_DEG;
    
    attitude_log_file << timestamp << " " << msgid << " "
                     << std::fixed << std::setprecision(2)
                     << roll_deg << " " << pitch_deg << " " << yaw_deg << std::endl;
    
    attitude_log_file.flush();
}

// Convert quaternion to roll, pitch, yaw angles (in degrees)
Eigen::Vector3d quaternion_to_euler_angles(const Eigen::Quaterniond& q) {
    // Extract roll, pitch, yaw from quaternion
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); // ZYX order
    
    // Convert to degrees
    euler *= RAD_TO_DEG;
    
    return euler;
}

// MAVLink message handling thread
void mavlink_thread(int fd) {
    mavlink_status_t status;
    mavlink_message_t msg;
    
    while (running) {
        // Send heartbeat periodically (every 1 second)
        static time_t last_heartbeat_time = 0;
        time_t current_time = time(NULL);
        if (current_time - last_heartbeat_time >= 1) {
            send_heartbeat(fd);
            last_heartbeat_time = current_time;
        }
        
        // Read from serial port
        uint8_t byte;
        ssize_t bytes_read = read(fd, &byte, 1);
        
        if (bytes_read > 0) {
            // Try to parse a MAVLink message from the byte
            if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
                // Handle different message types
                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
                        mavlink_local_position_ned_t lpos;
                        mavlink_msg_local_position_ned_decode(&msg, &lpos);
                        
                        // Log position data to file
                        log_position_data(lpos, msg.msgid);
                        
                        // Update vehicle position
                        std::lock_guard<std::mutex> lock(vehicle_pose.mutex);
                        vehicle_pose.position_ned = Eigen::Vector3d(lpos.x, lpos.y, lpos.z);
                        vehicle_pose.timestamp = std::chrono::steady_clock::now();
                        vehicle_pose.valid = true;

                        // Store raw MAVLink values
                        std::lock_guard<std::mutex> data_lock(mavlink_data.mutex);
                        mavlink_data.lpos_x = lpos.x;
                        mavlink_data.lpos_y = lpos.y;
                        mavlink_data.lpos_z = lpos.z;
                        mavlink_data.valid = true;
                        break;
                    }
                    
                    case MAVLINK_MSG_ID_ATTITUDE: {
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&msg, &attitude);
                        
                        // Log attitude data to file
                        log_attitude_data(attitude, msg.msgid);
                        
                        // Convert euler angles to quaternion
                        Eigen::AngleAxisd rollAngle(attitude.roll, Eigen::Vector3d::UnitX());
                        Eigen::AngleAxisd pitchAngle(attitude.pitch, Eigen::Vector3d::UnitY());
                        Eigen::AngleAxisd yawAngle(attitude.yaw, Eigen::Vector3d::UnitZ());
                        
                        // Update vehicle orientation
                        std::lock_guard<std::mutex> lock(vehicle_pose.mutex);
                        vehicle_pose.orientation = yawAngle * pitchAngle * rollAngle;
                        vehicle_pose.timestamp = std::chrono::steady_clock::now();
                        vehicle_pose.valid = true;

                        // Store raw MAVLink values
                        std::lock_guard<std::mutex> data_lock(mavlink_data.mutex);
                        mavlink_data.attitude_roll = attitude.roll;
                        mavlink_data.attitude_pitch = attitude.pitch;
                        mavlink_data.attitude_yaw = attitude.yaw;
                        mavlink_data.valid = true;
                        break;
                    }
                }
            }
        }
        
        // Small delay to prevent CPU hogging
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// Structure to hold ArUco tag in different reference frames
struct ArucoTag {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
    std::chrono::steady_clock::time_point timestamp;
    int id;
    
    bool valid() const {
        return !std::isnan(position.x());
    }
};

class ArucoDetector {
public:
    ArucoDetector() {
        // Initialize camera matrix from provided values
        camera_matrix_ = (cv::Mat_<double>(3,3) << 
            1.263093565868901123e+03, 0.000000000000000000e+00, 3.373397936510543786e+02,
            0.000000000000000000e+00, 1.262837893820984846e+03, 2.984055022766331717e+02,
            0.000000000000000000e+00, 0.000000000000000000e+00, 1.000000000000000000e+00);
        
        // Initialize distortion coefficients
        dist_coeffs_ = (cv::Mat_<double>(1,5) << 
            1.330578835447483177e-01, -2.700494534630855092e+00, 
            3.580030869520298316e-03, -1.483281586745981368e-03, 
            2.747394534267536059e+01);
        
        // Setup ArUco detector for DICT_4X4_250
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        detector_params_ = cv::aruco::DetectorParameters::create();
        
        // Define object points for the marker (in meters)
        // ArUco marker with size 0.373 meters
        float half_size = marker_size_ / 2.0f;
        object_points_ = {
            cv::Point3f(-half_size,  half_size, 0),  // top left
            cv::Point3f( half_size,  half_size, 0),  // top right
            cv::Point3f( half_size, -half_size, 0),  // bottom right
            cv::Point3f(-half_size, -half_size, 0)   // bottom left
        };
    }
    
    void processFrame(uint8_t* rgb_data, int width, int height) {
        // Convert RGB data to OpenCV Mat
        cv::Mat frame(height, width, CV_8UC3, rgb_data);
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
        
        // Detect markers
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(gray, dictionary_, corners, ids, detector_params_);
        
        // Look for marker ID 0
        bool marker_found = false;
        for (size_t i = 0; i < ids.size(); i++) {
            if (ids[i] == target_marker_id_) {
                // Found marker ID 0, estimate pose
                estimatePose(corners[i], ids[i]);
                marker_found = true;
                
                // Add console feedback for marker detection
                auto now = std::chrono::steady_clock::now();
                std::string timestamp = format_timestamp(now);
                std::cout << "\033[1;32m[" << timestamp << "] Marker ID " << ids[i] 
                        << " detected!\033[0m" << std::endl;
                break;
            }
        }
        
        // If you want less frequent updates when nothing is found, you can use a counter
        static int frame_counter = 0;
        if (!marker_found) {
            frame_counter++;
            if (frame_counter >= 60) {  // Only print every 60 frames
                frame_counter = 0;
                auto now = std::chrono::steady_clock::now();
                std::string timestamp = format_timestamp(now);
                std::cout << "[" << timestamp << "] No markers detected" << std::endl;
            }
        } else {
            frame_counter = 0;  // Reset counter when a marker is found
        }
    }
    
private:
    void estimatePose(const std::vector<cv::Point2f>& corners, int marker_id) {
        // Undistort corner points
        std::vector<cv::Point2f> undistorted_corners;
        cv::undistortPoints(corners, undistorted_corners, camera_matrix_, dist_coeffs_, 
                           cv::noArray(), camera_matrix_);
        
        // Solve PnP to get rotation and translation vectors
        cv::Vec3d rvec, tvec;
        cv::solvePnP(object_points_, undistorted_corners, camera_matrix_, 
                     cv::noArray(), rvec, tvec);
        
        // Calculate rotation angle (magnitude of rotation vector)
        double angle = cv::norm(rvec);
        
        // Normalize rotation vector to get rotation axis
        Eigen::Vector3d axis;
        if (angle > 0.0001) {  // Avoid division by zero
            axis = Eigen::Vector3d(rvec[0]/angle, rvec[1]/angle, rvec[2]/angle);
        } else {
            axis = Eigen::Vector3d(0, 0, 1);  // Default axis if angle is near zero
        }
        
        // Create quaternion directly from axis-angle representation
        Eigen::Quaterniond q_camera(Eigen::AngleAxisd(angle, axis));
        
        // Create position vector
        Eigen::Vector3d p_camera(tvec[0], tvec[1], tvec[2]);
        
        // Create tag in camera frame
        ArucoTag tag_camera = {
            .position = p_camera,
            .orientation = q_camera,
            .timestamp = std::chrono::steady_clock::now(),
            .id = marker_id
        };
        
        // Transform to world frame
        ArucoTag tag_world = getTagWorld(tag_camera);
        
        // Log tag orientation
        logTagOrientation(tag_world);
        
        // Send LANDING_TARGET message
        sendLandingTargetMessage(tag_world);
    }
    
    // Send LANDING_TARGET message to PX4
    void sendLandingTargetMessage(const ArucoTag& tag) {
        if (global_mavlink_fd < 0 || !tag.valid()) return;
        
        // Get current timestamp in microseconds since system boot
        uint64_t time_usec = std::chrono::duration_cast<std::chrono::microseconds>(
            tag.timestamp.time_since_epoch()).count();
        
        // Extract position and orientation information
        float x = tag.position.x();
        float y = tag.position.y();
        float z = tag.position.z();
        
        // Extract quaternion components
        float q_w = tag.orientation.w();
        float q_x = tag.orientation.x();
        float q_y = tag.orientation.y();
        float q_z = tag.orientation.z();
        
        // Send the LANDING_TARGET message
        send_landing_target(global_mavlink_fd, time_usec, tag.id, x, y, z, q_w, q_x, q_y, q_z);
        
        // Print debug info
        Eigen::Vector3d euler = quaternion_to_euler_angles(tag.orientation);
        std::cout << "Sent LANDING_TARGET message - Position: (" << x << ", " << y << ", " << z << ") m, "
                 << "Orientation (rpy): (" << euler[0] << ", " << euler[1] << ", " << euler[2] << ") deg" << std::endl;
    }
    
    // Log tag orientation data (position and roll/pitch/yaw) to dedicated file
    void logTagOrientation(const ArucoTag& tag) {
        if (!tag_orientation_log_file.is_open()) return;

        // Calculate roll, pitch, yaw from quaternion
        Eigen::Vector3d euler = quaternion_to_euler_angles(tag.orientation);
        
        // Get timestamp
        std::string timestamp = format_timestamp(tag.timestamp);
        
        // Write to tag orientation log file
        tag_orientation_log_file << timestamp << " " << tag.id << " "
                               << std::fixed << std::setprecision(6)
                               << tag.position.x() << " " 
                               << tag.position.y() << " " 
                               << tag.position.z() << " "
                               << std::fixed << std::setprecision(2)
                               << euler[0] << " "  // Roll in degrees
                               << euler[1] << " "  // Pitch in degrees
                               << euler[2]         // Yaw in degrees
                               << std::endl;
        
        tag_orientation_log_file.flush();
    }
    
    ArucoTag getTagWorld(const ArucoTag& tag) {
        // Get vehicle pose data
        Eigen::Vector3d vehicle_position;
        Eigen::Quaterniond vehicle_orientation;
        
        {
            std::lock_guard<std::mutex> lock(vehicle_pose.mutex);
            if (!vehicle_pose.valid) {
                // If no valid vehicle pose is available, return the tag in camera frame
                return tag;
            }
            vehicle_position = vehicle_pose.position_ned;
            vehicle_orientation = vehicle_pose.orientation;
        }
        
        // Convert from optical to NED (similar to PrecisionLand.cpp)
        // Optical: X right, Y down, Z away from lens
        // NED: X forward, Y right, Z down
        Eigen::Matrix3d R;
        R << -1,  0, 0,
              0, -1, 0,
              0,  0, 1;
        Eigen::Quaterniond quat_NED(R);

        // Camera offset in drone's FRD frame
        Eigen::Vector3d camera_offset(0, -0.01, 0.1);  // Forward, Right, Down
        
        // Create transforms
        Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
        Eigen::Affine3d camera_transform = Eigen::Translation3d(camera_offset) * quat_NED;
        Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
        
        // Compute world transform
        Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;
        
        // Extract position and orientation
        ArucoTag world_tag = {
            .position = tag_world_transform.translation(),
            .orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
            .timestamp = tag.timestamp,
            .id = tag.id
        };
        
        return world_tag;
    }
    
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    std::vector<cv::Point3f> object_points_;
    const float marker_size_ = 0.373f;  // meters
    const int target_marker_id_ = 0;
};

int main() {
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    
    // Set libcamera log level to reduce verbosity
    logSetLevel("*", "ERROR");
    
    // Generate timestamp for log files
    std::time_t now = std::time(nullptr);
    std::tm* timeinfo = std::localtime(&now);
    char timestamp[20];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", timeinfo);
    
    // Set global start time
    global_start_time = std::chrono::steady_clock::now();
    
    // Initialize log files
    if (!init_log_files(std::string(timestamp))) {
        std::cerr << "Failed to initialize log files, exiting" << std::endl;
        return 1;
    }
    
    // Initialize MAVLink communication
    int mavlink_fd = serial_init(SERIAL_PORT, BAUDRATE);
    if (mavlink_fd < 0) {
        std::cerr << "Failed to initialize MAVLink communication" << std::endl;
        close_log_files();
        return 1;
    }
    
    // Set global MAVLink file descriptor for other threads
    global_mavlink_fd = mavlink_fd;
    
    // Start MAVLink thread
    std::thread mavlink_thread_handle(mavlink_thread, mavlink_fd);
    
    // Request data streams from PX4
    request_data_stream(mavlink_fd, MAV_DATA_STREAM_POSITION, 10);  // Position at 10 Hz
    request_data_stream(mavlink_fd, MAV_DATA_STREAM_EXTRA1, 10);   // Attitude at 10 Hz
    
    // Initialize camera manager
    CameraManager camera_manager;
    camera_manager.start();
    
    // Get first camera
    auto cameras = camera_manager.cameras();
    if (cameras.empty()) {
        std::cerr << "No cameras found" << std::endl;
        running = 0;
        mavlink_thread_handle.join();
        close(mavlink_fd);
        close_log_files();
        return 1;
    }
    
    auto camera = cameras[0];
    std::cout << "Using camera: " << camera->id() << std::endl;
    
    if (camera->acquire()) {
        std::cerr << "Failed to acquire camera" << std::endl;
        running = 0;
        mavlink_thread_handle.join();
        close(mavlink_fd);
        close_log_files();
        return 1;
    }
    
    // Configure camera
    std::unique_ptr<CameraConfiguration> config = camera->generateConfiguration({ StreamRole::Viewfinder });
    config->at(0).pixelFormat = formats::RGB888;
    config->at(0).size = Size(640, 480);
    config->at(0).bufferCount = 4;  // Use more buffers for smoother operation
    
    // Validate and apply configuration
    CameraConfiguration::Status validation = config->validate();
    if (validation == CameraConfiguration::Invalid) {
        std::cerr << "Camera configuration invalid" << std::endl;
        camera->release();
        running = 0;
        mavlink_thread_handle.join();
        close(mavlink_fd);
        close_log_files();
        return 1;
    }
    
    if (camera->configure(config.get())) {
        std::cerr << "Failed to configure camera" << std::endl;
        camera->release();
        running = 0;
        mavlink_thread_handle.join();
        close(mavlink_fd);
        close_log_files();
        return 1;
    }
    
    // Allocate buffers
    FrameBufferAllocator allocator(camera);
    Stream *stream = config->at(0).stream();
    if (allocator.allocate(stream) < 0) {
        std::cerr << "Failed to allocate buffers" << std::endl;
        camera->release();
        running = 0;
        mavlink_thread_handle.join();
        close(mavlink_fd);
        close_log_files();
        return 1;
    }
    
    std::cout << "Allocated " << allocator.buffers(stream).size() << " buffers" << std::endl;
    
    // Create requests
    std::vector<std::unique_ptr<Request>> requests;
    for (const auto &buffer : allocator.buffers(stream)) {
        std::unique_ptr<Request> request = camera->createRequest();
        if (!request) {
            std::cerr << "Failed to create request" << std::endl;
            allocator.free(stream);
            camera->release();
            running = 0;
            mavlink_thread_handle.join();
            close(mavlink_fd);
            close_log_files();
            return 1;
        }
        
        if (request->addBuffer(stream, buffer.get())) {
            std::cerr << "Failed to add buffer to request" << std::endl;
            allocator.free(stream);
            camera->release();
            running = 0;
            mavlink_thread_handle.join();
            close(mavlink_fd);
            close_log_files();
            return 1;
        }
        
        requests.push_back(std::move(request));
    }
    
    // Start camera
    if (camera->start()) {
        std::cerr << "Failed to start camera" << std::endl;
        allocator.free(stream);
        camera->release();
        running = 0;
        mavlink_thread_handle.join();
        close(mavlink_fd);
        close_log_files();
        return 1;
    }
    
    std::cout << "Camera started successfully. Press Ctrl+C to exit." << std::endl;
    std::cout << "Sending LANDING_TARGET messages to PX4..." << std::endl;
    
    try {
        // Create ArUco detector
        ArucoDetector aruco_detector;
        
        // Queue all requests initially
        for (auto& request : requests) {
            if (camera->queueRequest(request.get()) < 0) {
                std::cerr << "Failed to queue request" << std::endl;
            }
        }
        
        // Continuous capture loop
        int frame_count = 0;
        auto start_time = std::chrono::steady_clock::now();
        
        while (running) {
            // Find a completed request
            Request* completed_request = nullptr;
            for (auto& request : requests) {
                if (request->status() == Request::RequestComplete) {
                    completed_request = request.get();
                    break;
                }
            }
            
            if (!completed_request) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            
            // Process frame
            const FrameBuffer *buffer = completed_request->buffers().at(stream);
            const FrameBuffer::Plane &plane = buffer->planes()[0];
            void *data = mmap(nullptr, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
            
            if (data != MAP_FAILED) {
                // Process with ArUco detector
                aruco_detector.processFrame(static_cast<uint8_t*>(data), 
                                           config->at(0).size.width, 
                                           config->at(0).size.height);
                
                munmap(data, plane.length);
                frame_count++;
                
                // Print progress every second
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
                if (frame_count % 30 == 0) {
                    std::cout << "Processed " << frame_count << " frames (" << elapsed << " seconds)" << std::endl;
                }
            }
            
            // Reuse and requeue the request
            completed_request->reuse(Request::ReuseBuffers);
            camera->queueRequest(completed_request);
        }
        
        std::cout << "\nProcessed " << frame_count << " frames total" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    
    // Cleanup
    std::cout << "Cleaning up..." << std::endl;
    camera->stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Give camera time to stop
    requests.clear();
    allocator.free(stream);
    camera->release();
    
    // Shutdown MAVLink communication
    running = 0;
    mavlink_thread_handle.join();
    close(mavlink_fd);
    
    // Close log files
    close_log_files();
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Give system time to process release
    camera_manager.stop();
    
    return 0;
}