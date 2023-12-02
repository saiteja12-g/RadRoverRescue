/**
 * @file Manipulation.hpp
 * @author Akash Parmar (akasparm.umd.edu)
 * @brief 
 * @version 0.1
 * @date 2023-12-02
 * 
 * @copyright Copyright (c) 2023
 * 
 */

// Ensures that the header file is included only once during compilation
#pragma once

#include "rclcpp/rclcpp.hpp" // ROS 2 client library for C++
#include "geometry_msgs/msg/pose.hpp" // Message type for specifying a position and orientation in space
#include "gazebo_msgs/srv/spawn_entity.hpp" // Service for spawning an entity in Gazebo
#include "gazebo_msgs/srv/delete_entity.hpp" // Service for deleting an entity in Gazebo
#include <gazebo_msgs/srv/detail/delete_entity__struct.hpp> // Detailed structure definition for the delete entity service
#include <fstream> // File stream library for C++
#include <iostream> // Standard library for input-output streams

// Aliases for ease of use and readability
using REQUEST_DELETE = gazebo_msgs::srv::DeleteEntity::Request;
using SERVICE_DELETE = gazebo_msgs::srv::DeleteEntity;
using CLIENT_DELETE = rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr;
using RESPONSE_DELETE = rclcpp::Client<SERVICE_DELETE>::SharedFuture;

using REQUEST_SPAWN = gazebo_msgs::srv::SpawnEntity::Request;
using SERVICE_SPAWN = gazebo_msgs::srv::SpawnEntity;
using CLIENT_SPAWN = rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr;
using RESPONSE_SPAWN = rclcpp::Client<SERVICE_DELETE>::SharedFuture;

using namespace std::chrono_literals; // Allows the use of time literals (e.g., 10s for 10 seconds)
using std::placeholders::_1; // For use in std::bind, to refer to the first argument of a bound function

// Definition of the Manipulation class
class Manipulation : public rclcpp::Node {
 public:
    Manipulation();
    // Constructor for the class

    bool pick_obj();
    // Method to pick an object - returns true if successful

    bool place_obj();
    // Method to place an object - returns true if successful

    void gripper_open();
    // Method to open the gripper

    void gripper_close();
    // Method to close the gripper

 private:
    geometry_msgs::msg::Pose pick_pose;
    // Pose for picking the object

    geometry_msgs::msg::Pose place_pose;
    // Pose for placing the object
};
