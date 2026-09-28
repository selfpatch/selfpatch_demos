#!/bin/bash
# Inject LiDAR sensor failure - high failure probability
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

if ! curl -sf -X PUT "${API_BASE}/apps/lidar-driver/configurations/failure_probability" \
    -H "Content-Type: application/json" -d '{"value": 0.8}' > /dev/null 2>&1; then
    echo "FAIL: lidar-driver/failure_probability"
    exit 1
fi

echo '{"status": "injected", "parameter": "failure_probability", "value": 0.8}'
