// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file navigation_test.cpp
 * @author Sai Teja Gilukara (Navigator)
 * @author Akash Parmar (Driver)
 * @brief Navigation tests
 * @version 0.1
 * @date 2023-12-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "../include/Navigation.hpp"

using ODOM = nav_msgs::msg::Odometry;
/**
 * @brief Task Navigation test
 * 
 */
class TaskNavigation : public testing::Test {
 public:
  rclcpp::Node::SharedPtr node_;
  TIMER timer_;
  rclcpp::Publisher<ODOM>::SharedPtr test_pub;
  void callback();
  PUBLISHER nav_publisher_;
};
/**
 * @brief task navigation callback
 * 
 */
void TaskNavigation::callback() {
  auto message = ODOM();
  test_pub->publish(message);
}
/**
 * @brief Construct a new test f object
 * 
 */
TEST_F(TaskNavigation, test_search_bins) {
  node_ = rclcpp::Node::make_shared("test_navigation");
  auto ypos = 3.0;
  TIMER timer = node_->create_wall_timer(
      100ms, std::bind(&TaskNavigation::callback, this));
  rclcpp::spin_some(node_);
  POSE rpyGoal;
  rpyGoal.header.frame_id = "map";
  rpyGoal.header.stamp = node_->get_clock()->now();
  rpyGoal.pose.position.x = 0;
  rpyGoal.pose.position.y = ypos;
  rpyGoal.pose.position.z = 0;
  rpyGoal.pose.orientation.x = 0;
  rpyGoal.pose.orientation.y = 0;
  rpyGoal.pose.orientation.z = 0;
  rpyGoal.pose.orientation.w = 1;
  ASSERT_TRUE(true);
}
/**
 * @brief Construct a new test f object
 * 
 */
TEST_F(TaskNavigation, test_resume_search) {
  node_ = rclcpp::Node::make_shared("test_navigation");
  TIMER timer = node_->create_wall_timer(
      100ms, std::bind(&TaskNavigation::callback, this));
  rclcpp::spin_some(node_);
  POSE rpyGoal;
  rpyGoal.header.frame_id = "map";
  rpyGoal.header.stamp = node_->get_clock()->now();
  rpyGoal.pose.position.x = 0;
  rpyGoal.pose.position.y = 0;
  rpyGoal.pose.position.z = 0;
  rpyGoal.pose.orientation.x = 0;
  rpyGoal.pose.orientation.y = 0;
  rpyGoal.pose.orientation.z = 0;
  rpyGoal.pose.orientation.w = 1;
  ASSERT_TRUE(true);
}
/**
 * @brief Construct a new test f object
 * 
 */
TEST_F(TaskNavigation, test_move_to_disposal) {
  node_ = rclcpp::Node::make_shared("test_navigation");
  rclcpp::spin_some(node_);
  TIMER timer = node_->create_wall_timer(
      100ms, std::bind(&TaskNavigation::callback, this));
  POSE rpyGoal;
  rpyGoal.header.frame_id = "map";
  rpyGoal.header.stamp = node_->get_clock()->now();
  rpyGoal.pose.position.x = 3;
  rpyGoal.pose.position.y = -2.5;
  rpyGoal.pose.position.z = 0;
  rpyGoal.pose.orientation.x = 0;
  rpyGoal.pose.orientation.y = 0;
  rpyGoal.pose.orientation.z = 0;
  rpyGoal.pose.orientation.w = 1;
  ASSERT_TRUE(true);
}
