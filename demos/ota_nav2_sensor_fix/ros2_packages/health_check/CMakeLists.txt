cmake_minimum_required(VERSION 3.16)
project(health_check)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(health_check_node src/health_check_node.cpp)
ament_target_dependencies(health_check_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_srvs
)

install(TARGETS health_check_node DESTINATION lib/${PROJECT_NAME})

ament_package()
