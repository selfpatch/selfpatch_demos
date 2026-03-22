#!/bin/bash
# Inject high noise: noise_stddev=0.5 on lidar-sim and noise_level=0.3 on camera-sim
set -e

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

curl -s -X PUT "${API_BASE}/apps/lidar-sim/configurations/noise_stddev" \
    -H "Content-Type: application/json" -d '{"value": 0.5}'
echo "lidar-sim: noise_stddev=0.5"

curl -s -X PUT "${API_BASE}/apps/camera-sim/configurations/noise_level" \
    -H "Content-Type: application/json" -d '{"value": 0.3}'
echo "camera-sim: noise_level=0.3"

echo "High noise injected on lidar-sim and camera-sim"
exit 0
