cmake_minimum_required(VERSION 3.8)
project(sensor_diagnostics_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)

# LiDAR simulator node
add_executable(lidar_sim_node src/lidar_sim_node.cpp)
ament_target_dependencies(lidar_sim_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
  rcl_interfaces
)

# IMU simulator node
add_executable(imu_sim_node src/imu_sim_node.cpp)
ament_target_dependencies(imu_sim_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

# GPS simulator node
add_executable(gps_sim_node src/gps_sim_node.cpp)
ament_target_dependencies(gps_sim_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
)

# Camera simulator node
add_executable(camera_sim_node src/camera_sim_node.cpp)
ament_target_dependencies(camera_sim_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
  rcl_interfaces
)

# Anomaly detector node
add_executable(anomaly_detector_node src/anomaly_detector_node.cpp)
ament_target_dependencies(anomaly_detector_node
  rclcpp
  sensor_msgs
  diagnostic_msgs
  ros2_medkit_msgs
)

# Install executables
install(TARGETS
  lidar_sim_node
  imu_sim_node
  gps_sim_node
  camera_sim_node
  anomaly_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
