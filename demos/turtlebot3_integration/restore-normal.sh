#!/bin/bash
# Restore Normal Operation - Reset all parameters and clear faults
# Use this after running any inject-*.sh script

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🔄 Restoring NORMAL operation..."
echo ""

# Check for jq dependency
if ! command -v jq >/dev/null 2>&1; then
    echo "❌ 'jq' is required but not installed."
    echo "   Please install jq (e.g., 'sudo apt-get install jq') and retry."
    exit 1
fi

# Check gateway
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    exit 1
fi

# Cancel any active navigation goals
echo "Canceling any active navigation goals..."
# List active executions and cancel them
EXECUTIONS=$(curl -s "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" 2>/dev/null)
if echo "$EXECUTIONS" | jq -e '.items[]' > /dev/null 2>&1; then
    echo "$EXECUTIONS" | jq -r '.items[].id' | while read -r EXEC_ID; do
        if [ -n "$EXEC_ID" ]; then
            curl -s -X DELETE "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/$EXEC_ID" > /dev/null 2>&1
            echo "  Canceled execution: $EXEC_ID"
        fi
    done
fi

# Restore velocity smoother defaults
echo "Restoring velocity_smoother defaults..."
curl -s -X PUT "${API_BASE}/apps/velocity-smoother/configurations/max_velocity" \
  -H "Content-Type: application/json" \
  -d '{"value": [0.26, 0.0, 1.0]}' > /dev/null

# Restore controller defaults
echo "Restoring controller_server defaults..."
curl -s -X PUT "${API_BASE}/apps/controller-server/configurations/FollowPath.max_vel_x" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.26}' > /dev/null

# Clear all faults
echo ""
echo "Clearing all faults from FaultManager..."
curl -s -X DELETE "${API_BASE}/faults" > /dev/null

echo ""
echo "✓ Normal operation restored!"
echo ""
echo "Current fault status:"
curl -s "${API_BASE}/faults" | jq '.items | length' | xargs -I {} echo "  Active faults: {}"
echo ""
echo "Robot is ready for normal operation."
