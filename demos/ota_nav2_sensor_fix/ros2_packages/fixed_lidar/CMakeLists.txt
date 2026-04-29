cmake_minimum_required(VERSION 3.16)
project(fixed_lidar)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)

add_executable(fixed_lidar_node src/fixed_lidar_node.cpp)
ament_target_dependencies(fixed_lidar_node rclcpp sensor_msgs ros2_medkit_msgs)

install(TARGETS fixed_lidar_node DESTINATION lib/${PROJECT_NAME})

ament_package()
