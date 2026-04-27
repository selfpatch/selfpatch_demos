#!/usr/bin/env bash
# Copyright 2026 bburda
# Apache 2.0
#
# Container entrypoint: hands off to the ota_nav2_sensor_fix_demo launch file
# which orchestrates everything (TB3 + Nav2 + headless Gazebo + foxglove_bridge
# + fault_manager + broken_lidar/legacy + gateway w/ ota_update_plugin).

set -e

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash

# Default to headless; an operator on a workstation can flip via env var.
HEADLESS_ARG="${HEADLESS:-true}"

exec ros2 launch ota_nav2_sensor_fix_demo demo.launch.py \
  "headless:=${HEADLESS_ARG}"
