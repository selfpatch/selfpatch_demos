#!/bin/bash
# Reset all actuation node parameters to defaults
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

# Motor controller
put_config motor-controller torque_noise 0.01
put_config motor-controller failure_probability 0.0

# Joint driver
put_config joint-driver inject_overheat false
put_config joint-driver drift_rate 0.0
put_config joint-driver failure_probability 0.0

# Gripper controller
put_config gripper-controller inject_jam false
put_config gripper-controller failure_probability 0.0

if [ $ERRORS -gt 0 ]; then
    echo "{\"status\": \"partial\", \"errors\": $ERRORS}"
    exit 1
fi

# Clear faults
echo "Clearing faults..."
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true
sleep 2
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true

echo '{"status": "restored", "ecu": "actuation"}'
