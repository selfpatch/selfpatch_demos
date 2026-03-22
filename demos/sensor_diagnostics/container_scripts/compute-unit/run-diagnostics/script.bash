#!/bin/bash
# Check health of all 4 sensors by querying their data endpoints and reporting active faults
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SENSORS=("lidar-sim/data/scan" "imu-sim/data/imu" "gps-sim/data/fix" "camera-sim/data/image")
ERRORS=0

for sensor_path in "${SENSORS[@]}"; do
    app=$(echo "$sensor_path" | cut -d/ -f1)
    echo "Checking $app..."
    if ! curl -sf "${API_BASE}/apps/${sensor_path}" > /dev/null 2>&1; then
        echo "  FAIL: No response from $app"
        ERRORS=$((ERRORS + 1))
    else
        echo "  OK: $app responding"
    fi
done

# Also check faults
FAULT_COUNT=$(curl -sf "${API_BASE}/faults" | jq '.items | length' 2>/dev/null || echo "?")
echo "Active faults: $FAULT_COUNT"

if [ $ERRORS -gt 0 ]; then
    echo "DIAGNOSTICS FAILED: $ERRORS sensor(s) not responding"
    exit 1
fi

echo "All sensors healthy"
exit 0
