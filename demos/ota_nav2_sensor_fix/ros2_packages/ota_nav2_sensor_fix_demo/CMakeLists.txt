cmake_minimum_required(VERSION 3.16)
project(ota_nav2_sensor_fix_demo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()
