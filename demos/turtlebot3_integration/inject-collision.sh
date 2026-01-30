#!/bin/bash
# Inject Collision Warning - Send robot toward obstacle
# This will trigger collision avoidance and potential path replanning

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "💥 Injecting COLLISION WARNING fault..."
echo "   Sending robot toward known obstacle location in turtlebot3_world"
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

# In turtlebot3_world, there are obstacles around the center
# Send goal that requires navigating close to obstacles
echo "Sending navigation goal to obstacle-dense area..."
RESPONSE=$(curl -s -X POST "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" \
  -H "Content-Type: application/json" \
  -d '{
    "request": {
      "pose": {
        "header": {"frame_id": "map"},
        "pose": {
          "position": {"x": 0.0, "y": 0.0, "z": 0.0},
          "orientation": {"w": 1.0}
        }
      }
    }
  }')

echo "$RESPONSE" | jq '.' 2>/dev/null || echo "$RESPONSE"

echo ""
echo "✓ Collision scenario triggered!"
echo ""
echo "Expected faults (via diagnostic_bridge):"
echo "  - COLLISION_MONITOR: Obstacle detected"
echo "  - CONTROLLER_SERVER: Path blocked"
echo ""
echo "The collision monitor should stop the robot if it gets too close."
echo "Check faults with: curl ${API_BASE}/faults | jq"
