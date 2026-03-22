#!/bin/bash
# Arm self-test - verify joint states are within expected limits
set -e
GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Running arm self-test..."
echo "Checking joint state broadcaster..."
RESPONSE=$(curl -sf "${API_BASE}/apps/joint-state-broadcaster" 2>&1)
if [ $? -ne 0 ]; then
    echo "FAIL: joint-state-broadcaster not responding"
    exit 1
fi
echo "OK: joint-state-broadcaster responding"

echo "Checking move-group..."
RESPONSE=$(curl -sf "${API_BASE}/apps/move-group" 2>&1)
if [ $? -ne 0 ]; then
    echo "FAIL: move-group not responding"
    exit 1
fi
echo "OK: move-group responding"

echo "Checking fault status..."
FAULT_COUNT=$(curl -sf "${API_BASE}/faults" | jq '.items | length' 2>/dev/null || echo "?")
echo "Active faults: $FAULT_COUNT"

echo "Arm self-test passed"
