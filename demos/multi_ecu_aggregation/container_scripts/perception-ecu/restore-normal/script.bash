#!/bin/bash
# Reset all perception node parameters to defaults
set -eu

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash

ERRORS=0

# LiDAR driver
ros2 param set /perception/lidar_driver failure_probability 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/lidar_driver inject_nan false || ERRORS=$((ERRORS + 1))
ros2 param set /perception/lidar_driver noise_stddev 0.01 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/lidar_driver drift_rate 0.0 || ERRORS=$((ERRORS + 1))

# Camera driver
ros2 param set /perception/camera_driver failure_probability 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/camera_driver noise_level 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/camera_driver inject_black_frames false || ERRORS=$((ERRORS + 1))

# Point cloud filter
ros2 param set /perception/point_cloud_filter failure_probability 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/point_cloud_filter drop_rate 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/point_cloud_filter delay_ms 0 || ERRORS=$((ERRORS + 1))

# Object detector
ros2 param set /perception/object_detector failure_probability 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/object_detector false_positive_rate 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /perception/object_detector miss_rate 0.0 || ERRORS=$((ERRORS + 1))

if [ $ERRORS -gt 0 ]; then
    echo "{\"status\": \"partial\", \"errors\": $ERRORS}"
    exit 1
fi
echo '{"status": "restored", "ecu": "perception"}'
