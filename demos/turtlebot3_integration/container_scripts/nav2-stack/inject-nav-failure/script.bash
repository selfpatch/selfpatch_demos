#!/bin/bash
# Inject navigation failure: send goal to unreachable location far outside map bounds
set -e

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Sending navigation goal to (100.0, 100.0) - far outside map..."
RESPONSE=$(curl -s -X POST "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" \
    -H "Content-Type: application/json" \
    -d '{
    "goal": {
      "pose": {
        "header": {"frame_id": "map"},
        "pose": {
          "position": {"x": 100.0, "y": 100.0, "z": 0.0},
          "orientation": {"w": 1.0}
        }
      }
    }
  }')

echo "${RESPONSE}" | jq '.' 2>/dev/null || echo "${RESPONSE}"

EXEC_ID=$(echo "${RESPONSE}" | jq -r '.execution_id // .id // empty' 2>/dev/null)

if [ -n "${EXEC_ID}" ] && [ "${EXEC_ID}" != "null" ]; then
    echo ""
    echo "Waiting for navigation to fail (checking status)..."
    for _ in $(seq 1 10); do
        sleep 2
        STATUS=$(curl -s "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/${EXEC_ID}" \
            | jq -r '.status' 2>/dev/null)
        echo "  Status: ${STATUS}"
        if [ "${STATUS}" = "failed" ] || [ "${STATUS}" = "completed" ]; then
            break
        fi
    done
fi

echo ""
echo "Navigation failure injected."
echo "Monitor faults at: ${API_BASE}/faults"
exit 0
