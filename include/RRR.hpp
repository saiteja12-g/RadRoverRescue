/**
 * @file RRR.hpp
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
#include "rclcpp/rclcpp.hpp"  // Main header for ROS 2 C++ functionality

// Include the header files for the Navigation, Perception, and Manipulation
// classes
#include "./Manipulation.hpp"
#include "./Navigation.hpp"
#include "./Perception.hpp"

/**
 * @brief Definition of the RRR class
 * 
 */
class RRR {
 public:
  /**
   * @brief Construct a new RRR object
   * 
   */
  RRR();
  /**
   * @brief Navigation stack
   * 
   */
  Navigation nav;
  /**
   * @brief Perception stack
   * 
   */
  Perception perc;
  /**
   * @brief Manipulation stack
   * 
   */
  Manipulation manip;
};
