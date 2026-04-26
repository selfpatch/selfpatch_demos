cmake_minimum_required(VERSION 3.16)
project(obstacle_classifier_v2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(obstacle_classifier_node src/obstacle_classifier_node.cpp)
ament_target_dependencies(obstacle_classifier_node rclcpp sensor_msgs visualization_msgs)

install(TARGETS obstacle_classifier_node DESTINATION lib/${PROJECT_NAME})

ament_package()
