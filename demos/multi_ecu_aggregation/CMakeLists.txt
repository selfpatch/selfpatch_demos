cmake_minimum_required(VERSION 3.8)
project(multi_ecu_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)

# --- Perception ECU nodes ---
add_executable(lidar_driver src/perception/lidar_driver.cpp)
ament_target_dependencies(lidar_driver rclcpp sensor_msgs diagnostic_msgs rcl_interfaces)

add_executable(camera_driver src/perception/camera_driver.cpp)
ament_target_dependencies(camera_driver rclcpp sensor_msgs diagnostic_msgs rcl_interfaces)

add_executable(point_cloud_filter src/perception/point_cloud_filter.cpp)
ament_target_dependencies(point_cloud_filter rclcpp sensor_msgs diagnostic_msgs rcl_interfaces)

add_executable(object_detector src/perception/object_detector.cpp)
ament_target_dependencies(object_detector rclcpp sensor_msgs vision_msgs diagnostic_msgs rcl_interfaces)

# --- Planning ECU nodes ---
add_executable(path_planner src/planning/path_planner.cpp)
ament_target_dependencies(path_planner rclcpp nav_msgs vision_msgs diagnostic_msgs rcl_interfaces)

add_executable(behavior_planner src/planning/behavior_planner.cpp)
ament_target_dependencies(behavior_planner rclcpp nav_msgs geometry_msgs diagnostic_msgs rcl_interfaces)

add_executable(task_scheduler src/planning/task_scheduler.cpp)
ament_target_dependencies(task_scheduler rclcpp std_msgs diagnostic_msgs rcl_interfaces)

# --- Actuation ECU nodes ---
add_executable(motor_controller src/actuation/motor_controller.cpp)
ament_target_dependencies(motor_controller rclcpp sensor_msgs geometry_msgs diagnostic_msgs rcl_interfaces)

add_executable(joint_driver src/actuation/joint_driver.cpp)
ament_target_dependencies(joint_driver rclcpp sensor_msgs diagnostic_msgs rcl_interfaces)

add_executable(gripper_controller src/actuation/gripper_controller.cpp)
ament_target_dependencies(gripper_controller rclcpp sensor_msgs geometry_msgs diagnostic_msgs rcl_interfaces)

# --- Install ---
install(TARGETS
  lidar_driver camera_driver point_cloud_filter object_detector
  path_planner behavior_planner task_scheduler
  motor_controller joint_driver gripper_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
