#!/bin/bash
# Reset all actuation node parameters to defaults and clear faults
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

ERRORS=0

# Sets one parameter through this ECU's gateway. A refused write is named on
# stderr, which the Scripts API returns as the error message.
put_config() {
    local app="$1" param="$2" value="$3" code
    code=$(curl -s -m 30 -o /dev/null -w '%{http_code}' -X PUT \
        "${API_BASE}/apps/${app}/configurations/${param}" \
        -H "Content-Type: application/json" -d "{\"value\": ${value}}") || true
    case "$code" in
        2??) echo "${app}: ${param}=${value}" ;;
        *)
            echo "FAIL: ${app}/${param} (HTTP ${code})" >&2
            ERRORS=$((ERRORS + 1))
            ;;
    esac
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

if [ "$ERRORS" -gt 0 ]; then
    echo "${ERRORS} parameter write(s) failed" >&2
    exit 1
fi

# Clears the faults of this ECU's fault manager. Prints the HTTP status.
clear_faults() {
    curl -s -m 30 -o /dev/null -w '%{http_code}' -X DELETE "${API_BASE}/faults" || true
}

# The second clear decides the result.
echo "Clearing faults..."
clear_faults > /dev/null
sleep 2
code=$(clear_faults)
case "$code" in
    2??) ;;
    *)
        echo "FAIL: clear faults (HTTP ${code})" >&2
        exit 1
        ;;
esac

echo '{"status": "restored", "ecu": "actuation"}'
