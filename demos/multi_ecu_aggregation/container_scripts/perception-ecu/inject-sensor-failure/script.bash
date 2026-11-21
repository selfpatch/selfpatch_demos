#!/bin/bash
# Inject LiDAR sensor failure - high failure probability
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

put_config lidar-driver failure_probability 0.8

if [ "$ERRORS" -gt 0 ]; then
    echo "${ERRORS} parameter write(s) failed" >&2
    exit 1
fi

echo '{"status": "injected", "parameter": "failure_probability", "value": 0.8}'
