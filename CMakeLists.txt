cmake_minimum_required(VERSION 3.8)
project(RadRoverRescue)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  set(TESTFILES
    test/main.cpp
    test/manipulation_test.cpp
    test/navigation_test.cpp
    test/perception_test.cpp
  )

  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES} include/Manipulation.hpp include/Navigation.hpp include/Perception.hpp)
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs tf2 tf2_ros)

  install(TARGETS
    test_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}  
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
