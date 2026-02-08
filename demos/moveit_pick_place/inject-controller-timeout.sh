#!/bin/bash
# Inject Controller Timeout - Set extremely tight goal time tolerance
# This will cause the arm controller to timeout during trajectory execution
#
# NOTE: This injection has no effect with fake/mock hardware (ros2_control
# FakeSystem) because the simulated controller reports instant success.
# It produces real faults only with physical robots or Gazebo physics.

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting CONTROLLER TIMEOUT fault..."
echo "   Setting extremely tight goal time tolerance (0.001s)"
echo "   This forces the controller to abort — it can't reach the target in time"
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

# Set controller parameters via gateway REST API (reliable, no DDS issues)
echo "Setting controller goal_time constraint via gateway API..."
curl -s -X PUT "${API_BASE}/apps/panda-arm-controller/configurations/constraints.goal_time" \
  -H "Content-Type: application/json" \
  -d '{"data": 0.001}' > /dev/null

curl -s -X PUT "${API_BASE}/apps/panda-arm-controller/configurations/constraints.stopped_velocity_tolerance" \
  -H "Content-Type: application/json" \
  -d '{"data": 0.0001}' > /dev/null

echo ""
echo "✓ Controller timeout injected!"
echo ""
echo "⚠️  NOTE: With fake hardware, the controller succeeds instantly and this"
echo "   injection will NOT produce faults. Use with real robot or Gazebo."
echo ""
echo "Expected faults (with real hardware):"
echo "  - CONTROLLER_TIMEOUT: Joint trajectory controller timed out"
echo "  - TRAJECTORY_EXECUTION_FAILED: Arm controller ABORTED trajectory"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
