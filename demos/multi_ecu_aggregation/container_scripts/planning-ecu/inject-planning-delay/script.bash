#!/bin/bash
# Inject path planning delay - 5000ms processing time
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

put_config path-planner planning_delay_ms 5000

if [ "$ERRORS" -gt 0 ]; then
    echo "${ERRORS} parameter write(s) failed" >&2
    exit 1
fi

echo '{"status": "injected", "parameter": "planning_delay_ms", "value": 5000}'
