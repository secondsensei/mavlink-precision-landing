# MAVLink Precision Landing System Summary

This program implements a vision-based precision landing system for drones running PX4 autopilot, using a Raspberry Pi as a companion computer with a camera to detect ArUco markers.

## System Architecture

The system consists of three primary components:

1. **Camera Interface** - Uses libcamera to capture frames from the Raspberry Pi camera
2. **ArUco Detection** - Processes images to detect and estimate the pose of ArUco markers 
3. **MAVLink Communication** - Exchanges messages with the PX4 flight controller

## Core Functionality

- **Image Acquisition**: The program configures and controls the Raspberry Pi camera using the libcamera framework, capturing frames at regular intervals.

- **ArUco Processing**: Each captured frame is analyzed for ArUco markers. When detected, the system computes the marker's 3D position and orientation relative to the camera.

- **Coordinate Transformation**: The marker position in camera coordinates is transformed into the drone's local NED (North-East-Down) coordinate frame using the vehicle's position and attitude information received from PX4.

- **LANDING_TARGET Messages**: The system sends MAVLink LANDING_TARGET messages to the PX4 autopilot, providing:
  - Target position (x, y, z) in the local NED frame
  - Target orientation as a quaternion
  - Target ID and validity flags

- **Data Logging**: The system logs position, attitude, and marker detection data to separate files for later analysis.

## Control Flow

1. The program initializes communication with the PX4 autopilot and sets up camera parameters
2. It launches a dedicated thread to handle MAVLink messages in the background
3. The main loop continuously captures images, processes them for ArUco markers, and sends landing target information when markers are detected
4. The system uses C++ mutexes to ensure thread-safe access to shared vehicle pose data

## Key Features

- Real-time logging of telemetry and detection data
- Coordinate frame conversions for precise target positioning
- Automatic shutdown on system signals for graceful termination

This system enhances landing precision by providing the autopilot with accurate landing target positions, enabling autonomous precision landing capabilities on PX4-based drones.
