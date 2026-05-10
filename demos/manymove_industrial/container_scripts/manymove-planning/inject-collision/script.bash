#!/bin/bash
# Trigger a real collision fault by flipping the BT blackboard's
# collision_detected flag. MoveManipulatorAction::onStart will see the flag
# next tick, emit MANYMOVE_PLANNER_COLLISION_DETECTED and return FAILURE.
#
# `set -u` is intentionally avoided: ROS 2's setup.bash references unbound
# variables when sourced from a fresh shell.
set -e

source /opt/ros/jazzy/setup.bash
source /opt/manymove_ws/install/setup.bash

# Default xArm7 prefix in the upstream launch is empty; the BT keys live in
# the global blackboard under the "<robot_prefix>collision_detected" name.
# update_blackboard expects value_data as quoted strings even for bool/double
# (the service serialises every entry as a string).
PREFIX="${ROBOT_PREFIX:-}"

ros2 service call /update_blackboard \
  manymove_msgs/srv/SetBlackboardValues \
  "{key: ['${PREFIX}collision_detected'], value_type: ['bool'], value_data: ['true']}"
