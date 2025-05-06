CXX = g++
CXXFLAGS = -Wall -O2 -std=c++17
PKG_CONFIG = pkg-config

# Libraries for libcamera
LIBCAMERA_CFLAGS = $(shell $(PKG_CONFIG) --cflags libcamera)
LIBCAMERA_LIBS = $(shell $(PKG_CONFIG) --libs libcamera)

# Libraries for OpenCV
OPENCV_CFLAGS = $(shell $(PKG_CONFIG) --cflags opencv4)
OPENCV_LIBS = $(shell $(PKG_CONFIG) --libs opencv4)

# Add Eigen
EIGEN_CFLAGS = -I/usr/include/eigen3
EIGEN_LIBS = 

# MAVLink include directory
MAVLINK_CFLAGS = -I$(HOME)/include

# Combine all flags for the mavlink-aruco-logger
MAVLINK_ARUCO_CFLAGS = $(LIBCAMERA_CFLAGS) $(OPENCV_CFLAGS) $(EIGEN_CFLAGS) $(MAVLINK_CFLAGS)
MAVLINK_ARUCO_LIBS = $(LIBCAMERA_LIBS) $(OPENCV_LIBS) $(EIGEN_LIBS) -lpthread -lm

# Program
TARGET = mavlink-precision-landing

all: $(TARGET)

$(TARGET): mavlink-precision-landing.cpp
	$(CXX) $(CXXFLAGS) $(MAVLINK_ARUCO_CFLAGS) -o $@ $< $(MAVLINK_ARUCO_LIBS)

clean:
	rm -f $(TARGET)

.PHONY: all clean