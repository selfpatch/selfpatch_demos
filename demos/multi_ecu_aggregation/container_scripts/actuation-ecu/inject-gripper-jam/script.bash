#!/bin/bash
# Inject gripper jam - gripper controller stuck
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

if ! curl -sf -X PUT "${API_BASE}/apps/gripper-controller/configurations/inject_jam" \
    -H "Content-Type: application/json" -d '{"value": true}' > /dev/null 2>&1; then
    echo "FAIL: gripper-controller/inject_jam"
    exit 1
fi

echo '{"status": "injected", "parameter": "inject_jam", "value": true}'
