/**
 * @file Perception.hpp
 * @author Akash Parmar (akasparm@umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-12-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

// Ensures that the header is included only once during compilation
#pragma once

#include "rclcpp/rclcpp.hpp" // Main header for ROS 2 C++ functionality
#include <opencv2/opencv.hpp> // Main header for the OpenCV library
#include "opencv2/core/core.hpp" // OpenCV core functionality
#include "opencv2/imgproc/imgproc.hpp" // OpenCV image processing functions
#include "geometry_msgs/msg/pose.hpp" // ROS 2 message type for representing a pose (position and orientation)
#include "sensor_msgs/msg/laser_scan.hpp" // ROS 2 message type for laser scan data
#include "sensor_msgs/msg/image.hpp" // ROS 2 message type for image data
#include "image_transport/image_transport.hpp" // ROS 2 library for image transport
#include "cv_bridge/cv_bridge.h" // ROS 2 library for converting between ROS image messages and OpenCV images
#include "opencv2/highgui/highgui.hpp" // OpenCV functionality for high-level GUI
#include "rclcpp/logging.hpp" // ROS 2 logging functionality
#include <memory> // Include for smart pointer utilities
#include <iostream> // Standard input-output stream
#include <vector> // Include for using the vector container
#include <chrono> // Include for time-related functions
#include <iomanip> // Include for input and output formatting
#include "geometry_msgs/msg/twist.hpp" // ROS 2 message type for representing velocities
#include "geometry_msgs/msg/pose_stamped.hpp" // ROS 2 message type for a pose with a timestamp
#include "nav_msgs/msg/odometry.hpp" // ROS 2 message type for representing odometry information

// Type aliases for convenience and readability
using TWIST = geometry_msgs::msg::Twist;
using std::placeholders::_1; // Used for binding arguments in callbacks
using std::chrono::duration; // For representing time durations
using namespace std::chrono_literals; // Allows using time literals like '10s'
using ODOM = nav_msgs::msg::Odometry;

// Definition of the Perception class
class Perception : public rclcpp::Node {
 public:
    Perception();
    // Constructor for the class

    bool detect_obj();
    // Method to detect objects - returns true if successful

    bool select_obj();
    // Method to select an object based on some criteria - returns true if successful

    void move_to_obj();
    // Method to initiate movement towards the selected object

 private:
    cv::Mat img_feed;
    // Image feed from a camera, stored as an OpenCV matrix

    sensor_msgs::msg::LaserScan lidar_feed;
    // Laser scan data, typically from a LiDAR sensor
};
