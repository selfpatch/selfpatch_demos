#!/bin/bash
# Cancel active navigation goals and reset AMCL global localization
set -e

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Canceling active navigation goals..."
EXECUTIONS=$(curl -sf "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" 2>/dev/null || echo '{"items":[]}')
if echo "${EXECUTIONS}" | jq -e '.items[]' > /dev/null 2>&1; then
    echo "${EXECUTIONS}" | jq -r '.items[].id' | while read -r EXEC_ID; do
        curl -sf -X DELETE "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/${EXEC_ID}" > /dev/null 2>&1 || true
        echo "  Canceled execution: ${EXEC_ID}"
    done
else
    echo "  No active executions found."
fi

echo ""
echo "Resetting AMCL global localization..."
curl -s -X POST "${API_BASE}/apps/amcl/operations/reinitialize_global_localization/executions" \
    -H "Content-Type: application/json" \
    -d '{}'

echo ""
echo "Navigation reset complete."
exit 0
