#!/bin/bash
# Inject path planning delay - 5000ms processing time
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

if ! curl -sf -X PUT "${API_BASE}/apps/path-planner/configurations/planning_delay_ms" \
    -H "Content-Type: application/json" -d '{"value": 5000}' > /dev/null 2>&1; then
    echo "FAIL: path-planner/planning_delay_ms"
    exit 1
fi

echo '{"status": "injected", "parameter": "planning_delay_ms", "value": 5000}'
