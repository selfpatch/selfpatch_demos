#!/bin/bash
# Reset all sensor parameters to defaults and clear all active faults
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Restoring all sensors to normal operation..."

# LiDAR
curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/noise_stddev" \
    -H "Content-Type: application/json" -d '{"value": 0.01}' || true
curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/failure_probability" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": false}' || true
curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/drift_rate" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
echo "lidar-sim: restored to defaults"

# IMU
curl -sf -X PUT "${API_BASE}/apps/imu-sim/configurations/accel_noise_stddev" \
    -H "Content-Type: application/json" -d '{"value": 0.01}' || true
curl -sf -X PUT "${API_BASE}/apps/imu-sim/configurations/failure_probability" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
curl -sf -X PUT "${API_BASE}/apps/imu-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": false}' || true
curl -sf -X PUT "${API_BASE}/apps/imu-sim/configurations/drift_rate" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
echo "imu-sim: restored to defaults"

# GPS
curl -sf -X PUT "${API_BASE}/apps/gps-sim/configurations/position_noise_stddev" \
    -H "Content-Type: application/json" -d '{"value": 2.0}' || true
curl -sf -X PUT "${API_BASE}/apps/gps-sim/configurations/failure_probability" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
curl -sf -X PUT "${API_BASE}/apps/gps-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": false}' || true
curl -sf -X PUT "${API_BASE}/apps/gps-sim/configurations/drift_rate" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
echo "gps-sim: restored to defaults"

# Camera
curl -sf -X PUT "${API_BASE}/apps/camera-sim/configurations/noise_level" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
curl -sf -X PUT "${API_BASE}/apps/camera-sim/configurations/failure_probability" \
    -H "Content-Type: application/json" -d '{"value": 0.0}' || true
curl -sf -X PUT "${API_BASE}/apps/camera-sim/configurations/inject_black_frames" \
    -H "Content-Type: application/json" -d '{"value": false}' || true
echo "camera-sim: restored to defaults"

# Clear all faults
curl -s -X DELETE "${API_BASE}/apps/diagnostic-bridge/faults/LIDAR_SIM" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/diagnostic-bridge/faults/CAMERA_SIM" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/diagnostic-bridge/faults/IMU_SIM" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/diagnostic-bridge/faults/GPS_SIM" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/anomaly-detector/faults/SENSOR_TIMEOUT" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/anomaly-detector/faults/SENSOR_NAN" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/anomaly-detector/faults/SENSOR_OUT_OF_RANGE" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/anomaly-detector/faults/RATE_DEGRADED" > /dev/null 2>&1 || true
curl -s -X DELETE "${API_BASE}/apps/anomaly-detector/faults/NO_FIX" > /dev/null 2>&1 || true
echo "All faults cleared"

echo "All sensors restored to normal operation"
exit 0
