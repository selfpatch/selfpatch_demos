#!/bin/bash
# Reset all planning node parameters to defaults and clear faults
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

# Path planner
put_config path-planner planning_delay_ms 0
put_config path-planner failure_probability 0.0

# Behavior planner
put_config behavior-planner inject_wrong_direction false
put_config behavior-planner failure_probability 0.0

# Task scheduler
put_config task-scheduler inject_stuck false
put_config task-scheduler failure_probability 0.0

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

echo '{"status": "restored", "ecu": "planning"}'
