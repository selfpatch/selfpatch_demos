#!/bin/bash
# Inject Navigation Failure - Send goal to unreachable location
# This will trigger a path planning failure from Nav2

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting NAVIGATION FAILURE fault..."
echo "   Sending goal to unreachable location (outside map bounds)"
echo ""

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
    "request": {
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

echo ""
echo "✓ Navigation failure injected!"
echo ""
echo "Expected faults (via diagnostic_bridge):"
echo "  - BT_NAVIGATOR: Goal rejected or path planning failed"
echo "  - PLANNER_SERVER: No valid path to goal"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
