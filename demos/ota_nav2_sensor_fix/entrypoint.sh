#!/usr/bin/env bash
# Copyright 2026 bburda
# Apache 2.0
#
# Container entrypoint: hands off to the ota_nav2_sensor_fix_demo launch file
# which orchestrates everything (RB-Theron AMR + Nav2 + headless Gazebo +
# foxglove_bridge + fault_manager + gateway w/ ota_update_plugin). Once the
# gateway is up, this script auto-applies broken_lidar_3_0_0 so the mission
# starts on the regressed lidar the operator has to diagnose.

set -e

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash

# Default to headless; an operator on a workstation can flip via env var.
HEADLESS_ARG="${HEADLESS:-true}"

# Simulate a routine software update that regressed the lidar: once the
# gateway is healthy and the plugin has registered the catalog, apply
# broken_lidar_3_0_0 so scan_sensor_node is running the bad build before
# the mission starts. The operator later finds it in /updates, publishes
# the forward hotfix (fixed_lidar_3_0_1) with publish-fix.sh, and applies
# it with apply-fix.sh.
(
  API="http://localhost:8080/api/v1"
  for _ in $(seq 1 60); do
    if curl -fsS "${API}/updates" 2>/dev/null | grep -q 'broken_lidar_3_0_0'; then break; fi
    sleep 2
  done
  curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' "${API}/updates/broken_lidar_3_0_0/prepare" >/dev/null 2>&1 || true
  sleep 3
  curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' "${API}/updates/broken_lidar_3_0_0/execute" >/dev/null 2>&1 || true
) &

exec ros2 launch ota_nav2_sensor_fix_demo demo.launch.py \
  "headless:=${HEADLESS_ARG}"
