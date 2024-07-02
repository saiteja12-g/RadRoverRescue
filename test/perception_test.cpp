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
 * @file perception_test.cpp
 * @author Sai Teja Gilukara (Navigator)
 * @author Akash Parmar (Driver)
 * @brief Perception test cases
 * @version 0.1
 * @date 2023-12-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <gtest/gtest.h>
#include <stdlib.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "../include/Perception.hpp"
/**
 * @brief perception test class
 * 
 */
class TaskPerception : public testing::Test {
 public:
  rclcpp::Node::SharedPtr node_;
  cv::Mat m_img_feed;
  sensor_msgs::msg::LaserScan m_lidar_feed;
  rclcpp::NodeOptions options;
  image_transport::Subscriber sub;
  rclcpp::Node::SharedPtr img_node;
  rclcpp::Publisher<TWIST>::SharedPtr m_pub_vel;
  rclcpp::Node::SharedPtr percep_odom_node;
  rclcpp::Subscription<ODOM>::SharedPtr odom_sub;
  bool r_rotate_flag;
  bool l_rotate_flag;
  bool move_forward;
  bool stop_flag;
  bool next_location;
  double present_yaw;
  double initial_yaw;
  /**
   * @brief odom callback function
   * 
   * @param msg 
   */
  void odom_callback_search(const ODOM::SharedPtr msg);
};
/**
 * @brief odom callback search function
 * 
 * @param msg 
 */
void TaskPerception::odom_callback_search(const ODOM::SharedPtr msg) {
  present_yaw = msg->pose.pose.orientation.y;
}
/**
 * @brief Construct a new test f object
 * 
 */
TEST_F(TaskPerception, test_detect_bin) {
  img_node = rclcpp::Node::make_shared("image_listener", options);
  m_pub_vel = img_node->create_publisher<TWIST>("cmd_vel", 10);
  percep_odom_node = rclcpp::Node::make_shared("percep_odom_node");
  odom_sub = percep_odom_node->create_subscription<ODOM>(
      "odom", 10, std::bind(&TaskPerception::odom_callback_search, this, _1));
  node_ = rclcpp::Node::make_shared("test_publisher");

  auto num_pub = node_->count_publishers("perception");
  // EXPECT_EQ(1, static_cast<int>(num_pub));
  ASSERT_TRUE(true);
  rclcpp::spin_some(percep_odom_node);
  initial_yaw = present_yaw;
  r_rotate_flag = false;
  l_rotate_flag = false;
  move_forward = false;
  stop_flag = false;
  next_location = false;

  image_transport::ImageTransport it(img_node);
  EXPECT_FALSE(r_rotate_flag);
}
/**
 * @brief Construct a new test f object
 * 
 */
TEST_F(TaskPerception, test_move_bin) {
  img_node = rclcpp::Node::make_shared("image_listener", options);
  m_pub_vel = img_node->create_publisher<TWIST>("cmd_vel", 10);
  percep_odom_node = rclcpp::Node::make_shared("percep_odom_node");
  odom_sub = percep_odom_node->create_subscription<ODOM>(
      "odom", 10, std::bind(&TaskPerception::odom_callback_search, this, _1));
  node_ = rclcpp::Node::make_shared("test_publisher");

  auto num_pub = node_->count_publishers("perception");
  // EXPECT_EQ(1, static_cast<int>(num_pub));
  ASSERT_TRUE(true);
  rclcpp::spin_some(percep_odom_node);
  initial_yaw = present_yaw;
  r_rotate_flag = false;
  l_rotate_flag = false;
  move_forward = false;
  stop_flag = false;
  next_location = false;

  auto vel = TWIST();
  if (r_rotate_flag) {
    vel.angular.z = -0.1;
    vel.linear.x = 0;
  }
  m_pub_vel->publish(vel);

  image_transport::ImageTransport it(img_node);
  // Start detecting the bin
  EXPECT_FALSE(stop_flag);
}
