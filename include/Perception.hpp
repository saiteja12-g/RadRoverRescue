/**
 * @file Perception.hpp
 * @author Akash Parmar - Driver
 * @author Sai Teja Gilukara - Navigator
 * @brief
 * @version 0.1
 * @date 2023-12-02
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <chrono>              // Include for time-related functions
#include <iomanip>             // Include for input and output formatting
#include <iostream>            // Standard input-output stream
#include <memory>              // Include for smart pointer utilities
#include <opencv2/opencv.hpp>  // Main header for the OpenCV library
#include <vector>              // Include for using the vector container

#include "cv_bridge/cv_bridge.h"  // ROS 2 library for converting between ROS image messages and OpenCV images
#include "geometry_msgs/msg/pose.hpp"  // ROS 2 message type for representing a pose (position and orientation)
#include "geometry_msgs/msg/pose_stamped.hpp"  // ROS 2 message type for a pose with a timestamp
#include "geometry_msgs/msg/twist.hpp"  // ROS 2 message type for representing velocities
#include "image_transport/image_transport.hpp"  // ROS 2 library for image transport
#include "nav_msgs/msg/odometry.hpp"  // ROS 2 message type for representing odometry information
#include "opencv2/core/core.hpp"        // OpenCV core functionality
#include "opencv2/highgui/highgui.hpp"  // OpenCV functionality for high-level GUI
#include "opencv2/imgproc/imgproc.hpp"  // OpenCV image processing functions
#include "rclcpp/logging.hpp"           // ROS 2 logging functionality
#include "rclcpp/rclcpp.hpp"          // Main header for ROS 2 C++ functionality
#include "sensor_msgs/msg/image.hpp"  // ROS 2 message type for image data
#include "sensor_msgs/msg/laser_scan.hpp"  // ROS 2 message type for laser scan data

// Type aliases for convenience and readability
using TWIST = geometry_msgs::msg::Twist;
using std::chrono::duration;  // For representing time durations
using std::placeholders::_1;  // Used for binding arguments in callbacks
using namespace std::chrono_literals;  // Allows using time literals like '10s'
using ODOM = nav_msgs::msg::Odometry;

/**
 * @brief Definition of the Perception class
 * 
 */
class Perception : public rclcpp::Node {
 public:
 /**
  * @brief Construct a new Perception object
  * 
  */
  Perception();
  // Constructor for the class
  /**
   * @brief Method to detect objects
   * 
   * @return true 
   * @return false 
   */
  bool detect_obj();

  /**
   * @brief move to object
   * 
   * @return true 
   * @return false 
   */
  bool move_to_obj();

  /**
   * @brief img callback function
   * 
   * @param msg 
   */
  void img_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  /**
   * @brief odom callback function
   * 
   * @param msg 
   */
  void odom_callback_search(const ODOM::SharedPtr msg);

 private:
  cv::Mat img_feed;
  // Image feed from a camera, stored as an OpenCV matrix

  sensor_msgs::msg::LaserScan lidar_feed;
  // Laser scan data, typically from a LiDAR sensor

  rclcpp::NodeOptions options;
  image_transport::Subscriber sub;
  rclcpp::Node::SharedPtr img_node;
  rclcpp::Publisher<TWIST>::SharedPtr pub_vel;
  rclcpp::Node::SharedPtr percep_odom_node;
  rclcpp::Subscription<ODOM>::SharedPtr odom_sub;
  bool r_rotate_flag;
  bool l_rotate_flag;
  bool move_forward;
  bool stop_flag;
  bool next_location;
  double present_yaw;
  double initial_yaw;


};
