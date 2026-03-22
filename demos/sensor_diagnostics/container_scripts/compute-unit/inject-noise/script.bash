#!/bin/bash
# Inject high noise: noise_stddev=0.5 on lidar-sim and noise_level=0.3 on camera-sim
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

ERRORS=0

if curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/noise_stddev" \
    -H "Content-Type: application/json" -d '{"value": 0.5}' > /dev/null 2>&1; then
    echo "lidar-sim: noise_stddev=0.5"
else
    echo "FAIL: lidar-sim"; ERRORS=$((ERRORS + 1))
fi

if curl -sf -X PUT "${API_BASE}/apps/camera-sim/configurations/noise_level" \
    -H "Content-Type: application/json" -d '{"value": 0.3}' > /dev/null 2>&1; then
    echo "camera-sim: noise_level=0.3"
else
    echo "FAIL: camera-sim"; ERRORS=$((ERRORS + 1))
fi

if [ $ERRORS -gt 0 ]; then
    echo "Noise injection partially failed: $ERRORS error(s)"
    exit 1
fi
echo "High noise injected on lidar-sim and camera-sim"
exit 0
