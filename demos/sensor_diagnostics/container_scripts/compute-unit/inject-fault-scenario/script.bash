#!/bin/bash
# Inject composite fault scenario: NaN values on lidar, IMU, GPS and black frames on camera simultaneously
set -e

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting composite fault scenario..."

curl -s -X PUT "${API_BASE}/apps/lidar-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}'
echo "lidar-sim: inject_nan=true"

curl -s -X PUT "${API_BASE}/apps/imu-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}'
echo "imu-sim: inject_nan=true"

curl -s -X PUT "${API_BASE}/apps/gps-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}'
echo "gps-sim: inject_nan=true"

curl -s -X PUT "${API_BASE}/apps/camera-sim/configurations/inject_black_frames" \
    -H "Content-Type: application/json" -d '{"value": true}'
echo "camera-sim: inject_black_frames=true"

echo "Composite fault scenario injected on all sensors"
exit 0
