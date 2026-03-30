#!/bin/bash
# Inject LiDAR sensor failure - high failure probability
set -eu

source /opt/ros/jazzy/setup.bash
source /root/demo_ws/install/setup.bash

ros2 param set /perception/lidar_driver failure_probability 0.8

echo '{"status": "injected", "parameter": "failure_probability", "value": 0.8}'
