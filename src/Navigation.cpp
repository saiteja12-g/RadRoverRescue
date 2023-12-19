/**
 * @file Navigation.cpp
 * @author Sai Teja Gilukara (Driver)
 * @author Akash Parmar (Navigator)
 * @brief 
 * @version 0.1
 * @date 2023-12-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "../include/Navigation.hpp"
#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

/**
 * @brief search for the closest waypoint
 * 
 * @param msg 
 */
void Navigation::search(const ODOM::SharedPtr msg) {
    if ((std::abs(static_cast<int>(msg->pose.pose.position.y - req_pos_y))
        == 0) && (std::abs(static_cast<int>(msg->pose.pose.position.x)) == 0)) {
            check_odom = true;
    }
}
/**
 * @brief disposal of trash
 * 
 * @param msg 
 */
void Navigation::disposal(const ODOM::SharedPtr msg) {
    if ((std::abs(static_cast<int>(msg->pose.pose.position.x - 3)) == 0)
        && (std::abs(static_cast<int>(msg->pose.pose.position.y + 2.5)) == 0)) {
            check_odom = true;
    }
}
/**
 * @brief Resuming search
 * 
 * @param msg 
 */
void Navigation::resume(const ODOM::SharedPtr msg) {
    if ((std::abs(static_cast<int>(msg->pose.pose.position.x)) == 0)
        && (std::abs(static_cast<int>(msg->pose.pose.position.y)) == 0)) {
            check_odom = true;
    }
}
/**
 * @brief Construct a new Navigation:: Navigation object
 * 
 */
Navigation::Navigation() : Node("navigation") {
    node_odom_nav = rclcpp::Node::
                    make_shared("odom_node");
    nav_publisher_ = this->create_publisher<POSE>("goal_pose", 10);
    check_odom = false;
    req_pos_y = 0.0;
}
/**
 * @brief Searching for object (path)
 * 
 * @return true 
 * @return false 
 */
bool Navigation::search_obj() {
    rclcpp::sleep_for(1000ms);
    std::vector<float_t> search_pos = {6.0, 4.0, 2.0, 0.0};
    auto odom_sub = node_odom_nav->create_subscription<ODOM>("odom", 10,
                        std::bind(&Navigation::search, this, _1));
    while (search_pos.size() > 0) {
        float_t pop_pos = search_pos.back();
        search_pos.pop_back();
        RCLCPP_INFO(this->get_logger(), "Searching for radioactive waste",
                    search_pos.size(), pop_pos);
        check_odom = false;
        req_pos_y = pop_pos;
        POSE rpyGoal;
        rpyGoal.header.frame_id = "map";
        rpyGoal.header.stamp = this->get_clock()->now();
        rpyGoal.pose.position.x = 0;
        rpyGoal.pose.position.y = pop_pos;
        rpyGoal.pose.position.z = 0;
        rpyGoal.pose.orientation.x = 0;
        rpyGoal.pose.orientation.y = 0;
        rpyGoal.pose.orientation.z = 0;
        rpyGoal.pose.orientation.w = 1;
        while (!check_odom) {
            rclcpp::spin_some(node_odom_nav);
            nav_publisher_->publish(rpyGoal);
            rclcpp::sleep_for(500ms);
        }
        return false;
    }
    return true;
}
/**
 * @brief move to disposal zone
 * 
 * @return true 
 * @return false 
 */
bool Navigation::move_to_disposal_zone() {
    RCLCPP_INFO(this->get_logger(), "Picked up waste, Moving towards track bin");
    check_odom = false;
    auto odom_sub = node_odom_nav->create_subscription<ODOM>("odom", 10,
                    std::bind(&Navigation::disposal, this, _1));
    POSE rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = this->get_clock()->now();
    rpyGoal.pose.position.x = 3;
    rpyGoal.pose.position.y = -2.5;
    rpyGoal.pose.position.z = 0;
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = 0;
    rpyGoal.pose.orientation.w = 1;
    while (!check_odom) {
        rclcpp::spin_some(node_odom_nav);
        nav_publisher_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
    }
    return true;
}
/**
 * @brief resuming search
 * 
 * @return true 
 * @return false 
 */
bool Navigation::resume_search() {
    RCLCPP_INFO(this->get_logger(), "Towards home position");
    check_odom = false;
    auto odom_sub = node_odom_nav->create_subscription<ODOM>("odom", 10,
                    std::bind(&Navigation::search, this, _1));
    POSE rpyGoal;
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = this->get_clock()->now();
    rpyGoal.pose.position.x = 0;
    rpyGoal.pose.position.y = 0;
    rpyGoal.pose.position.z = 0;
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = 0;
    rpyGoal.pose.orientation.w = 1;

    while (!check_odom) {
        rclcpp::spin_some(node_odom_nav);
        nav_publisher_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
    }
    return true;
}