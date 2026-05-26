#!/bin/bash
# Inject LiDAR sensor failure - high failure probability
set -eu

# ROS setup.bash dereferences AMENT_TRACE_SETUP_FILES; relax nounset around it.
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash
set -u

ros2 param set /perception/lidar_driver failure_probability 0.8

echo '{"status": "injected", "parameter": "failure_probability", "value": 0.8}'
