#!/bin/bash
# Inject sensor failure: set failure_probability=1.0 on imu-sim
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

curl -sf -X PUT "${API_BASE}/apps/imu-sim/configurations/failure_probability" \
    -H "Content-Type: application/json" -d '{"value": 1.0}'

echo "Failure injected: imu-sim failure_probability=1.0"
exit 0
