#!/bin/bash
# Inject path planning delay - 5000ms processing time
set -eu

# ROS setup.bash dereferences AMENT_TRACE_SETUP_FILES; relax nounset around it.
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash
set -u

ros2 param set /planning/path_planner planning_delay_ms 5000

echo '{"status": "injected", "parameter": "planning_delay_ms", "value": 5000}'
