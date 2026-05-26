#!/bin/bash
# Inject gripper jam - gripper controller stuck
set -eu

# ROS setup.bash dereferences AMENT_TRACE_SETUP_FILES; relax nounset around it.
set +u
# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash
set -u

ros2 param set /actuation/gripper_controller inject_jam true

echo '{"status": "injected", "parameter": "inject_jam", "value": true}'
