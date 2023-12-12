#include <gtest/gtest.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

#include "../include/Manipulation.hpp"

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
  EXPECT_EQ(1, static_cast<int>(num_pub));
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
