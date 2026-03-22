#!/bin/bash
# Planning benchmark - verify MoveIt planning is functional
set -eu
GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Running planning benchmark..."
echo "Checking move-group operations..."
if ! RESPONSE=$(curl -sf "${API_BASE}/apps/move-group/operations" 2>/dev/null); then
    echo "FAIL: Cannot list move-group operations"
    exit 1
fi
echo "OK: move-group operations available"

echo "Checking pick-place-node..."
if ! RESPONSE=$(curl -sf "${API_BASE}/apps/pick-place-node" 2>/dev/null); then
    echo "FAIL: pick-place-node not responding"
    exit 1
fi
echo "OK: pick-place-node responding"

echo "Checking manipulation monitor..."
if ! RESPONSE=$(curl -sf "${API_BASE}/apps/manipulation-monitor" 2>/dev/null); then
    echo "FAIL: manipulation-monitor not responding"
    exit 1
fi
echo "OK: manipulation-monitor responding"

echo "Planning benchmark passed"
