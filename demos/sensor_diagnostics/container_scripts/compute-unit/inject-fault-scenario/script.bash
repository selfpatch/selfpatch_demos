#!/bin/bash
# Inject composite fault scenario: NaN values on lidar, IMU, GPS and black frames on camera simultaneously
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting composite fault scenario..."
ERRORS=0

if curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}' > /dev/null 2>&1; then
    echo "lidar-sim: inject_nan=true"
else
    echo "FAIL: lidar-sim"; ERRORS=$((ERRORS + 1))
fi

if curl -sf -X PUT "${API_BASE}/apps/imu-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}' > /dev/null 2>&1; then
    echo "imu-sim: inject_nan=true"
else
    echo "FAIL: imu-sim"; ERRORS=$((ERRORS + 1))
fi

if curl -sf -X PUT "${API_BASE}/apps/gps-sim/configurations/inject_nan" \
    -H "Content-Type: application/json" -d '{"value": true}' > /dev/null 2>&1; then
    echo "gps-sim: inject_nan=true"
else
    echo "FAIL: gps-sim"; ERRORS=$((ERRORS + 1))
fi

if curl -sf -X PUT "${API_BASE}/apps/camera-sim/configurations/inject_black_frames" \
    -H "Content-Type: application/json" -d '{"value": true}' > /dev/null 2>&1; then
    echo "camera-sim: inject_black_frames=true"
else
    echo "FAIL: camera-sim"; ERRORS=$((ERRORS + 1))
fi

if [ $ERRORS -gt 0 ]; then
    echo "Composite injection partially failed: $ERRORS error(s)"
    exit 1
fi
echo "Composite fault scenario injected on all sensors"
exit 0
