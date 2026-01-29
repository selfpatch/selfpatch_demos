#!/bin/bash
# Inject Controller Failure - Set very restrictive velocity limits
# This will cause the controller to struggle following paths

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🎮 Injecting CONTROLLER FAILURE fault..."
echo "   Setting very restrictive velocity limits (robot will move very slowly)"
echo ""

# Check gateway
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    exit 1
fi

# Set very low max velocity on velocity_smoother
echo "Setting velocity_smoother max_velocity to 0.05 m/s (very slow)..."
curl -s -X PUT "${API_BASE}/apps/velocity-smoother/configurations/max_velocity" \
  -H "Content-Type: application/json" \
  -d '{"value": [0.05, 0.0, 0.3]}'

echo ""

# Also reduce controller's max velocity
echo "Setting controller_server FollowPath.max_vel_x to 0.05 m/s..."
curl -s -X PUT "${API_BASE}/apps/controller-server/configurations/FollowPath.max_vel_x" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.05}'

echo ""
echo "✓ Controller restriction injected!"
echo ""
echo "The robot will now move extremely slowly."
echo "Expected faults (via diagnostic_bridge):"
echo "  - VELOCITY_SMOOTHER: Velocity limited"
echo "  - CONTROLLER_SERVER: Path following degraded"
echo ""
echo "Restore with: ./restore-normal.sh"
echo "Check faults with: curl ${API_BASE}/faults | jq"
