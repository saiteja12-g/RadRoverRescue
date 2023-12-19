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

    bool search_obj();

    bool move_to_disposal_zone();

    bool resume_search();

    void search(const ODOM::SharedPtr msg);

    void disposal(const ODOM::SharedPtr msg);
 
    void resume(const ODOM::SharedPtr msg);

 private:
    PUBLISHER nav_publisher_;
    TIMER timer_;
    std::shared_ptr<rclcpp::Node> node_odom_nav;
    bool check_odom;
    float_t req_pos_y;
};
