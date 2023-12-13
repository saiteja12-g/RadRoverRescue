/**
 * @file Navigation.hpp
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

#include <chrono>    // Include for time-related functions
#include <iostream>  // Standard input-output stream
#include <memory>    // Include for smart pointer utilities
#include <vector>    // Include for using the vector container

#include "./Perception.hpp"  // Include the Perception header, presumably a custom class for perception tasks
#include "geometry_msgs/msg/pose.hpp"  // ROS 2 message type for representing a pose (position and orientation)
#include "geometry_msgs/msg/pose_stamped.hpp"  // ROS 2 message type for a pose with a timestamp
#include "nav_msgs/msg/odometry.hpp"  // ROS 2 message type for representing odometry information
#include "rclcpp/rclcpp.hpp"  // Main header for ROS 2 C++ functionality

// Type aliases for convenience and readability
using POSE = geometry_msgs::msg::PoseStamped;
using PUBLISHER = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;
using ODOM = nav_msgs::msg::Odometry;
using std::chrono::duration;  // For representing time durations
using std::placeholders::_1;  // Used for binding arguments in callbacks
using namespace std::chrono_literals;  // Allows using time literals like '10s'

// Definition of the Navigation class
class Navigation : public rclcpp::Node {
 public:
  Navigation();
  // Constructor for the class

  bool search_obj();
  // Method to search for an object - returns true if successful

  bool move_to_disposal_zone();
  // Method to move the robot to a disposal zone - returns true if successful

  bool resume_search();
  // Method to resume searching after an interruption - returns true if
  // successful

 private:
  geometry_msgs::msg::Pose current_pose;
  // Current pose of the robot

  geometry_msgs::msg::Pose next_pose;
  // Next target pose for the robot
};
