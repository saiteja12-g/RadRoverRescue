#include "../include/Manipulation.hpp"
#include <fstream>
#include <ios>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>

Manipulation::Manipulation() : Node("manipulation") {
    m_place_pose.position.x = 7;
    m_place_pose.position.y = -0.5;
    m_place_pose.position.z = 0;
    m_place_pose.orientation.x = 0;
    m_place_pose.orientation.y = 0;
    m_place_pose.orientation.z = 0;
    m_place_pose.orientation.w = 0;
    pick_client = create_client<SERVICE_DELETE>("delete_entity");
    place_client = create_client<SERVICE_SPAWN>("spawn_entity");
    manip_node = rclcpp::Node::
                    make_shared("bin_node");
}

bool Manipulation::pick_obj() {
    while (!pick_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(),
              "Interruped");

          return false;
        }
        RCLCPP_INFO(this->get_logger(),
              "Unavailable");
    }
    auto request = std::make_shared<REQUEST_DELETE>();
    request->name = "trash_bin1";

    auto result = pick_client->async_send_request(request);
    auto ret = rclcpp::spin_until_future_complete(manip_node,
                                            result, 10s);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    } else {
        return false;
    }
}

bool Manipulation::place_obj() {
    auto res = system("ros2 run gazebo_ros spawn_entity.py -entity trash_bin -x 3.5 -y -2.5 -z 0 -file `ros2 pkg prefix RadRoverRescue`/share/RadRoverRescue/models/bin_cylinder/model.sdf");
    if (res > 0) {
        return true;
    } else {
        return false;
    }
}