#!/bin/bash
# Restore normal operation: cancel goals, restore velocity params, clear all faults
set -e

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Canceling active navigation goals..."
EXECUTIONS=$(curl -s "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" 2>/dev/null || echo '{"items":[]}')
if echo "${EXECUTIONS}" | jq -e '.items[]' > /dev/null 2>&1; then
    echo "${EXECUTIONS}" | jq -r '.items[].id' | while read -r EXEC_ID; do
        if [ -n "${EXEC_ID}" ]; then
            curl -s -X DELETE "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/${EXEC_ID}" > /dev/null 2>&1 || true
            echo "  Canceled execution: ${EXEC_ID}"
        fi
    done
else
    echo "  No active executions found."
fi

echo ""
echo "Restoring velocity parameters to defaults..."
curl -s -X PUT "${API_BASE}/apps/velocity-smoother/configurations/max_velocity" \
    -H "Content-Type: application/json" \
    -d '{"value": [0.26, 0.0, 1.0]}' > /dev/null 2>&1 || true

curl -s -X PUT "${API_BASE}/apps/controller-server/configurations/FollowPath.max_vel_x" \
    -H "Content-Type: application/json" \
    -d '{"value": 0.26}' > /dev/null 2>&1 || true

echo "Clearing all faults..."
curl -s -X DELETE "${API_BASE}/faults" > /dev/null

echo ""
echo "Normal operation restored."
FAULT_COUNT=$(curl -sf "${API_BASE}/faults" | jq '.items | length' 2>/dev/null || echo "?")
echo "Active faults: ${FAULT_COUNT}"
exit 0
