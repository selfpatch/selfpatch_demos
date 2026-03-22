#!/bin/bash
# Planning benchmark - verify MoveIt planning is functional
set -e
GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Running planning benchmark..."
echo "Checking move-group operations..."
RESPONSE=$(curl -sf "${API_BASE}/apps/move-group/operations" 2>&1)
if [ $? -ne 0 ]; then
    echo "FAIL: Cannot list move-group operations"
    exit 1
fi
echo "OK: move-group operations available"

echo "Checking pick-place-node..."
RESPONSE=$(curl -sf "${API_BASE}/apps/pick-place-node" 2>&1)
if [ $? -ne 0 ]; then
    echo "FAIL: pick-place-node not responding"
    exit 1
fi
echo "OK: pick-place-node responding"

echo "Checking manipulation monitor..."
RESPONSE=$(curl -sf "${API_BASE}/apps/manipulation-monitor" 2>&1)
if [ $? -ne 0 ]; then
    echo "FAIL: manipulation-monitor not responding"
    exit 1
fi
echo "OK: manipulation-monitor responding"

echo "Planning benchmark passed"
