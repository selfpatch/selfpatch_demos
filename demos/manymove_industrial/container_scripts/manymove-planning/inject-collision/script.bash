#!/bin/bash
# Trigger a real collision fault by flipping the BT blackboard's
# collision_detected flag. MoveManipulatorAction::onStart will see the flag
# next tick, emit MANYMOVE_PLANNER_COLLISION_DETECTED and return FAILURE.
set -eu

source /opt/ros/jazzy/setup.bash
source /opt/manymove_ws/install/setup.bash

# Default xArm7 prefix in the upstream launch is empty; the BT keys live in
# the global blackboard under the "<robot_prefix>collision_detected" name.
PREFIX="${ROBOT_PREFIX:-}"

ros2 service call /hmi_service_node/update_blackboard \
  manymove_msgs/srv/SetBlackboardValues \
  "{key: ['${PREFIX}collision_detected'], value_type: ['bool'], value_data: ['true']}"
