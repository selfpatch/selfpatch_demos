#!/bin/bash
# Inject path planning delay - 5000ms processing time
set -eu

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash

ros2 param set /planning/path_planner planning_delay_ms 5000

echo '{"status": "injected", "parameter": "planning_delay_ms", "value": 5000}'
