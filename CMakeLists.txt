cmake_minimum_required(VERSION 3.8)
project(RadRoverRescue)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###################################
# find dependencies
###################################
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
# find_package(RadRoverRescue REQUIRED)

############################################
# Set up level 2 unit test (i.e, integration test)
############################################
if(COVERAGE)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS "--coverage")
  set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE "ON")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  set(TESTFILES
  test/main.cpp
  test/manipulation_test.cpp
  test/navigation_test.cpp
  test/perception_test.cpp
)
  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES} include/Manipulation.hpp include/Navigation.hpp include/Perception.hpp)
  target_include_directories(test_${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp rclcpp_action std_msgs geometry_msgs sensor_msgs image_transport cv_bridge OpenCV nav_msgs gazebo_msgs tf2 tf2_ros)
  install(TARGETS
    test_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}  
  )
  target_include_directories(test_${PROJECT_NAME} PUBLIC ../include/mario_com)
endif()

###################################################
# Create the compilation database for clangd and move it out of the build dir.
###################################################
# set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
# execute_process(COMMAND cmake -E create_symlink
#                 "${PROJECT_BINARY_DIR}/compile_commands.json"
#                 "${PROJECT_SOURCE_DIR}/compile_commands.json")
# ament_export_dependencies(rosidl_default_runtime)

# if (DOXYGEN_FOUND)
#   set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
#   set( DOXYGEN_COLLABORATION_GRAPH  YES )
#   set( DOXYGEN_EXTRACT_ALL          YES )
#   set( DOXYGEN_CLASS_DIAGRAMS       YES )
#   set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
#   set( DOXYGEN_HAVE_DOT             YES )
#   set( DOXYGEN_CLASS_GRAPH          YES )
#   set( DOXYGEN_CALL_GRAPH           YES )
#   set( DOXYGEN_CALLER_GRAPH         YES )
#   set( DOXYGEN_COLLABORATION_GRAPH  YES )
#   set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
#   set( DOXYGEN_EXTRACT_PRIVATE      YES )
#   set( DOXYGEN_EXTRACT_PACKAGE      YES )
#   set( DOXYGEN_EXTRACT_STATIC       YES )
#   set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
#   set( DOXYGEN_UML_LOOK             YES )
#   set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
#   set( DOXYGEN_TEMPLATE_RELATIONS   YES )
#   set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
#   set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
#   set( DOXYGEN_DOT_TRANSPARENT      YES )
# else()
#   message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
# endif()

###################################
# Specify Targets
###################################
# create a target to build documentation
# doxygen_add_docs(docs           # target name
#   # List of files or directories
#   ${PROJECT_SOURCE_DIR}/src
#   ${PROJECT_SOURCE_DIR}/include
#   ${PROJECT_SOURCE_DIR}/test
#   )
ament_package()
