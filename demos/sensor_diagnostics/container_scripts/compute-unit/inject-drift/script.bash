#!/bin/bash
# Inject sensor drift: set drift_rate=0.1 on lidar-sim
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

curl -sf -X PUT "${API_BASE}/apps/lidar-sim/configurations/drift_rate" \
    -H "Content-Type: application/json" -d '{"value": 0.1}'

echo "Drift injected: lidar-sim drift_rate=0.1"
exit 0
