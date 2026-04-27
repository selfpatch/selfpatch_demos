#!/usr/bin/env bash
# Copyright 2026 bburda
# Apache 2.0
#
# Container entrypoint: launches the demo nodes that the OTA plugin will
# manage at runtime, then forks the gateway as PID 1's foreground process.

set -e

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source /ws/install/setup.bash

# Demo nodes the plugin will swap (broken_lidar -> fixed_lidar) and
# uninstall (broken_lidar_legacy). obstacle_classifier_v2 is installed
# fresh by the demo and not started here.
ros2 run broken_lidar broken_lidar_node &
ros2 run broken_lidar_legacy broken_lidar_legacy &

# foxglove_bridge: WebSocket server on :8765 so Foxglove Studio can
# subscribe to /scan, /tf, and any topic the demo nodes publish. Required
# for the visual narrative (3D scene + phantom obstacle); the SOVD Updates
# panel itself only needs the gateway HTTP API.
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p port:=8765 -p address:=0.0.0.0 &

# Foreground gateway. Pass the config file directly to the gateway_node
# executable (the gateway.launch.py wrapper does not expose a config_file
# argument, so we invoke the executable directly to thread our YAML in).
exec ros2 run ros2_medkit_gateway gateway_node \
  --ros-args \
  --params-file /etc/ros2_medkit/gateway_config.yaml \
  --log-level info
