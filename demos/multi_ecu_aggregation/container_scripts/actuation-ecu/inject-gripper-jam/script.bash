#!/bin/bash
# Inject gripper jam - gripper controller stuck
set -eu

source /opt/ros/jazzy/setup.bash
source /root/demo_ws/install/setup.bash

ros2 param set /actuation/gripper_controller inject_jam true

echo '{"status": "injected", "parameter": "inject_jam", "value": true}'
