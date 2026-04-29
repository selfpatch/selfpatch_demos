cmake_minimum_required(VERSION 3.16)
project(broken_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)

add_executable(broken_lidar_node src/broken_lidar_node.cpp)
ament_target_dependencies(broken_lidar_node rclcpp sensor_msgs geometry_msgs ros2_medkit_msgs)

install(TARGETS broken_lidar_node DESTINATION lib/${PROJECT_NAME})

ament_package()
