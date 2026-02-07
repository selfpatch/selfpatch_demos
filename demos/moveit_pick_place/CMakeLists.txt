cmake_minimum_required(VERSION 3.8)
project(moveit_medkit_demo)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install scripts
install(PROGRAMS
  scripts/manipulation_monitor.py
  scripts/pick_place_loop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
