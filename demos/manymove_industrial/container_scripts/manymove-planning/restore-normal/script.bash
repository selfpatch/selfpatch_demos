#!/bin/bash
# Clear BT blackboard flags that the inject-* scripts may have set, so the
# next BT tick proceeds normally. Also resets the BT execution state so
# operators can rerun without restarting the demo.
set -e

source /opt/ros/jazzy/setup.bash
source /opt/manymove_ws/install/setup.bash

PREFIX="${ROBOT_PREFIX:-}"

ros2 service call /update_blackboard \
  manymove_msgs/srv/SetBlackboardValues \
  "{key: ['${PREFIX}collision_detected', '${PREFIX}stop_execution', '${PREFIX}reset', '${PREFIX}start'], \
    value_type: ['bool', 'bool', 'bool', 'bool'], \
    value_data: ['false', 'false', 'true', 'true']}"
