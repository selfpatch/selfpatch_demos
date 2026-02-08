#!/bin/bash
# Inject Joint Limit Violation - Command extreme joint positions
# This will trigger joint limit approaching/violation faults
#
# NOTE: While the pick-place loop is running, the controller will reject
# external trajectory goals (only one goal at a time). This injection
# works best when the pick-place loop is not active, or after stopping
# the mtc-pick-place node.

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting JOINT LIMIT fault..."
echo "   Commanding extreme joint positions near limits"
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

# Send a goal with extreme joint positions via gateway operations API
# These positions are near/beyond joint limits (see panda URDF)
echo "Sending joint trajectory goal near limits via gateway API..."
RESULT=$(curl -s -X POST "${API_BASE}/apps/panda-arm-controller/operations/follow_joint_trajectory/executions" \
  -H "Content-Type: application/json" \
  -d '{
    "input_data": {
      "trajectory": {
        "joint_names": ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"],
        "points": [{
          "positions": [2.85, 1.70, 2.85, -0.10, 2.85, 3.70, 2.85],
          "time_from_start": {"sec": 5, "nanosec": 0}
        }]
      }
    }
  }')

echo "  Result: ${RESULT}" | head -c 200
echo ""

echo ""
echo "✓ Joint limit fault injected!"
echo ""
echo "⚠️  NOTE: If the pick-place loop is running, the goal may be rejected"
echo "   (controller accepts only one goal at a time)."
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - JOINT_LIMIT_APPROACHING: Joint near limit (WARN)"
echo "  - JOINT_LIMIT_VIOLATED: Joint beyond limit (ERROR)"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
