#!/bin/bash
# Inject Navigation Failure - Send goal to unreachable location
# This will trigger a path planning failure from Nav2

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting NAVIGATION FAILURE fault..."
echo "   Sending goal to unreachable location (outside map bounds)"
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

# Send goal to location far outside the map (turtlebot3_world is small ~5x5m)
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

echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE"

# Extract execution ID
EXEC_ID=$(echo "$RESPONSE" | jq -r '.id' 2>/dev/null)

if [ -n "$EXEC_ID" ] && [ "$EXEC_ID" != "null" ]; then
    echo ""
    echo "Waiting for navigation to fail (checking status)..."
    for _ in {1..10}; do
        sleep 2
        STATUS=$(curl -s "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/${EXEC_ID}" | jq -r '.status' 2>/dev/null)
        echo "  Status: $STATUS"
        if [ "$STATUS" = "failed" ] || [ "$STATUS" = "completed" ]; then
            break
        fi
    done
fi

echo ""
echo "✓ Navigation failure injected!"
echo ""
echo "Expected faults (via anomaly_detector → FaultManager):"
echo "  - NAVIGATION_GOAL_ABORTED: Navigation goal ABORTED"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
