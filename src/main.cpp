/**
 * @file main.cpp
 * @author Sai Teja Gilukara (Driver)
 * @author Akash Parmar (Navigator)
 * @brief main function to call other stacks functions
 * @version 0.1
 * @date 2023-12-19
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <rclcpp/utilities.hpp>

#include "../include/RRR.hpp"
/**
 * @brief main function
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  RRR rrr;
  // Log a message indicating that the program is looking for RadioActive waste.
  RCLCPP_INFO(rclcpp::get_logger("log"), "Looking for RadioActive waste");

  // Perform a loop until the 'search_obj()' function returns true.
  while (!rrr.nav.search_obj()) {
    // Inside the loop, check if the 'detect_obj()' function returns true, and
    // break if it does.
    if (rrr.perc.detect_obj()) {
      break;
    }
  }
  rclcpp::sleep_for(2s);
  rrr.manip.pick_obj();
  rrr.nav.move_to_disposal_zone();
  rrr.manip.place_obj();
  rrr.nav.resume_search();
  rclcpp::shutdown();
  return 0;
}
