#!/bin/bash
# Reset all planning node parameters to defaults
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

ERRORS=0

# path-planner runs its planning cycle on a single-threaded executor and
# blocks it for the injected delay, so a write can land while the node is
# still busy from the last cycle. The gateway gives up on a busy node after
# a few seconds and holds it unavailable for a while after that, longer than
# the delay itself - so a short retry is not enough. Space retries out over
# a couple of minutes to reliably outlast that window.
put_config() {
    local app="$1" param="$2" value="$3"
    local tries_left=40
    while [ "$tries_left" -gt 0 ]; do
        if curl -sf -X PUT "${API_BASE}/apps/${app}/configurations/${param}" \
            -H "Content-Type: application/json" -d "{\"value\": ${value}}" > /dev/null 2>&1; then
            echo "${app}: ${param}=${value}"
            return 0
        fi
        tries_left=$((tries_left - 1))
        sleep 5
    done
    echo "FAIL: ${app}/${param}"
    ERRORS=$((ERRORS + 1))
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

if [ $ERRORS -gt 0 ]; then
    echo "{\"status\": \"partial\", \"errors\": $ERRORS}"
    exit 1
fi

# Clear faults
echo "Clearing faults..."
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true
sleep 2
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true

echo '{"status": "restored", "ecu": "planning"}'
