#!/bin/bash
# Inject NaN values on lidar-sim, imu-sim, and gps-sim simultaneously
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}'
echo "lidar-sim: inject_nan=true"

curl -sf -X PUT "${API_BASE}/apps/imu-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}'
echo "imu-sim: inject_nan=true"

curl -sf -X PUT "${API_BASE}/apps/gps-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}'
echo "gps-sim: inject_nan=true"

echo "NaN injection enabled on lidar-sim, imu-sim, gps-sim"
exit 0
