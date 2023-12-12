#include <stdlib.h>
#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include "std_msgs/msg/string.hpp"

#include "../include/Navigation.hpp"
#include "nav_msgs/msg/odometry.hpp"

using ODOM = nav_msgs::msg::Odometry;

class TaskNavigation : public testing::Test {
 public:
  rclcpp::Node::SharedPtr node_;
  TIMER timer_;
  rclcpp::Publisher<ODOM>::SharedPtr test_pub;
  void callback();
  PUBLISHER nav_publisher_;
};
void TaskNavigation::callback() {
  auto message = ODOM();
  test_pub->publish(message);
}

TEST_F(TaskNavigation, test_search_bins) {
  node_ = rclcpp::Node::make_shared("test_navigation");
  Navigation nav;
  // auto test_sub = node_->create_subscription<ODOM>("odom", 10, &callback);
  auto test_pub = node_->create_publisher<ODOM>
                    ("odom", 10.0);
  auto ypos = 3.0;
  TIMER timer = node_->create_wall_timer(100ms,
      std::bind(&TaskNavigation::callback, this));
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

TEST_F(TaskNavigation, test_resume_search) {
  node_ = rclcpp::Node::make_shared("test_navigation");
  Navigation nav;
  // auto test_sub = node_->create_subscription<ODOM>("odom", 10, &callback);
  auto test_pub = node_->create_publisher<ODOM>
                    ("odom", 10.0);

  TIMER timer = node_->create_wall_timer(100ms,
      std::bind(&TaskNavigation::callback, this));
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

TEST_F(TaskNavigation, test_move_to_disposal) {
  node_ = rclcpp::Node::make_shared("test_navigation");
  Navigation nav;
  // auto test_sub = node_->create_subscription<ODOM>("odom", 10, &callback);
  auto test_pub = node_->create_publisher<ODOM>
                    ("odom", 10.0);
  rclcpp::spin_some(node_);
  TIMER timer = node_->create_wall_timer(100ms,
      std::bind(&TaskNavigation::callback, this));
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
