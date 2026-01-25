#!/bin/bash
# Restore normal sensor operation (clear all faults)

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Restoring NORMAL operation..."

# LiDAR
echo "Resetting LiDAR parameters..."
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/noise_stddev" \
  -H "Content-Type: application/json" -d '{"value": 0.01}'
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/failure_probability" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/inject_nan" \
  -H "Content-Type: application/json" -d '{"value": false}'
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/drift_rate" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'

# IMU
echo "Resetting IMU parameters..."
curl -s -X PUT "${API_BASE}/apps/imu_sim/configurations/accel_noise_stddev" \
  -H "Content-Type: application/json" -d '{"value": 0.01}'
curl -s -X PUT "${API_BASE}/apps/imu_sim/configurations/failure_probability" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'
curl -s -X PUT "${API_BASE}/apps/imu_sim/configurations/inject_nan" \
  -H "Content-Type: application/json" -d '{"value": false}'
curl -s -X PUT "${API_BASE}/apps/imu_sim/configurations/drift_rate" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'

# GPS
echo "Resetting GPS parameters..."
curl -s -X PUT "${API_BASE}/apps/gps_sim/configurations/position_noise_stddev" \
  -H "Content-Type: application/json" -d '{"value": 2.0}'
curl -s -X PUT "${API_BASE}/apps/gps_sim/configurations/failure_probability" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'
curl -s -X PUT "${API_BASE}/apps/gps_sim/configurations/inject_nan" \
  -H "Content-Type: application/json" -d '{"value": false}'
curl -s -X PUT "${API_BASE}/apps/gps_sim/configurations/drift_rate" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'

# Camera
echo "Resetting Camera parameters..."
curl -s -X PUT "${API_BASE}/apps/camera_sim/configurations/noise_level" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'
curl -s -X PUT "${API_BASE}/apps/camera_sim/configurations/failure_probability" \
  -H "Content-Type: application/json" -d '{"value": 0.0}'
curl -s -X PUT "${API_BASE}/apps/camera_sim/configurations/inject_black_frames" \
  -H "Content-Type: application/json" -d '{"value": false}'

echo ""
echo "✓ Normal operation restored! All fault injections cleared."
