#!/bin/bash
# Drop a thin collision wall into the planning scene. MoveManipulatorAction
# will retry pick poses near it, occasionally fail planning, and emit
# MANYMOVE_PLANNER_RETRY_ATTEMPT (WARN) before either succeeding or hitting
# RETRIES_EXHAUSTED.
set -e

source /opt/ros/jazzy/setup.bash
source /opt/manymove_ws/install/setup.bash

ros2 action send_goal --feedback /add_collision_object \
  manymove_msgs/action/AddCollisionObject \
  "{object_id: 'soft_fault_wall', \
    primitive_type: 'box', \
    dimensions: [0.4, 0.02, 0.3], \
    pose: {position: {x: 0.35, y: 0.0, z: 0.15}, orientation: {w: 1.0}}, \
    frame_id: 'world', \
    color: {r: 1.0, a: 0.5}}" \
  || echo "(soft-fault: action server not available, skipping)"
