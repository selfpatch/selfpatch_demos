#!/bin/bash
# Reset all perception node parameters to defaults
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

ERRORS=0

put_config() {
    local app="$1" param="$2" value="$3"
    if curl -sf -X PUT "${API_BASE}/apps/${app}/configurations/${param}" \
        -H "Content-Type: application/json" -d "{\"value\": ${value}}" > /dev/null 2>&1; then
        echo "${app}: ${param}=${value}"
    else
        echo "FAIL: ${app}/${param}"
        ERRORS=$((ERRORS + 1))
    fi
}

# LiDAR driver
put_config lidar-driver failure_probability 0.0
put_config lidar-driver inject_nan false
put_config lidar-driver noise_stddev 0.01
put_config lidar-driver drift_rate 0.0

# Camera driver
put_config camera-driver failure_probability 0.0
put_config camera-driver noise_level 0.0
put_config camera-driver inject_black_frames false

# Point cloud filter
put_config point-cloud-filter failure_probability 0.0
put_config point-cloud-filter drop_rate 0.0
put_config point-cloud-filter delay_ms 0

# Object detector
put_config object-detector failure_probability 0.0
put_config object-detector false_positive_rate 0.0
put_config object-detector miss_rate 0.0

if [ $ERRORS -gt 0 ]; then
    echo "{\"status\": \"partial\", \"errors\": $ERRORS}"
    exit 1
fi

# Clear faults
echo "Clearing faults..."
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true
sleep 2
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true

echo '{"status": "restored", "ecu": "perception"}'
