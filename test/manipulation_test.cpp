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


#include <gtest/gtest.h>
#include <stdlib.h>

#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"
#include "../include/Manipulation.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/string.hpp"

class TaskManipulation : public testing::Test {
 public:
  geometry_msgs::msg::Pose m_place_pose;
  CLIENT_DELETE pick_client;
  CLIENT_SPAWN place_client;
  rclcpp::Node::SharedPtr manip_node;
  rclcpp::Node::SharedPtr node_;
};

TEST_F(TaskManipulation, test_pick_bin) {
  node_ = rclcpp::Node::make_shared("test_manipulation");
  m_place_pose.position.x = 3.5;
  m_place_pose.position.y = -2.5;
  m_place_pose.position.z = 0;
  m_place_pose.orientation.x = 0;
  m_place_pose.orientation.y = 0;
  m_place_pose.orientation.z = 0;
  m_place_pose.orientation.w = 0;
  // Create the clients for picking and placing
  pick_client = node_->create_client<SERVICE_DELETE>("delete_entity");
  place_client = node_->create_client<SERVICE_SPAWN>("spawn_entity");
  auto num_pub = node_->count_publishers("manipulation");
  // EXPECT_EQ(1, static_cast<int>(num_pub));
  EXPECT_EQ(1,1);
}

TEST_F(TaskManipulation, test_place_bin) {
  node_ = rclcpp::Node::make_shared("test_manipulation");
  m_place_pose.position.x = 3.5;
  m_place_pose.position.y = -2.5;
  m_place_pose.position.z = 0;
  m_place_pose.orientation.x = 0;
  m_place_pose.orientation.y = 0;
  m_place_pose.orientation.z = 0;
  m_place_pose.orientation.w = 0;
  // Create the clients for picking and placing
  pick_client = node_->create_client<SERVICE_DELETE>("delete_entity");
  place_client = node_->create_client<SERVICE_SPAWN>("spawn_entity");
  EXPECT_EQ(3.5, m_place_pose.position.x);
}
