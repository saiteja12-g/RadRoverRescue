#include "../include/RRR.hpp"
#include <rclcpp/utilities.hpp>
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    RRR rrr;
    RCLCPP_INFO(rclcpp::get_logger("log"), "Looking for RadioActive waste");
    while (!rrr.nav.search_obj()) {
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