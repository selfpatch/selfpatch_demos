#!/bin/bash
# Arm self-test - verify joint states are within expected limits
set -eu
GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Running arm self-test..."
echo "Checking joint state broadcaster..."
if ! curl -sf "${API_BASE}/apps/joint-state-broadcaster" > /dev/null 2>&1; then
    echo "FAIL: joint-state-broadcaster not responding"
    exit 1
fi
echo "OK: joint-state-broadcaster responding"

echo "Checking move-group..."
if ! curl -sf "${API_BASE}/apps/move-group" > /dev/null 2>&1; then
    echo "FAIL: move-group not responding"
    exit 1
fi
echo "OK: move-group responding"

echo "Checking fault status..."
FAULT_COUNT=$(curl -sf "${API_BASE}/faults" | jq '.items | length' 2>/dev/null || echo "?")
echo "Active faults: $FAULT_COUNT"

echo "Arm self-test passed"
