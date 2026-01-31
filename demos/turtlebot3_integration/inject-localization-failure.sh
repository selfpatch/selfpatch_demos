#!/bin/bash
# Inject Localization Failure - Reset AMCL with bad initial pose
# This will cause localization issues and potential navigation failures

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "📍 Injecting LOCALIZATION FAILURE fault..."
echo "   Reinitializing AMCL with incorrect pose (robot thinks it's somewhere else)"
echo ""

# Check gateway
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    exit 1
fi

# Reinitialize global localization - this scatters particles and causes uncertainty
echo "Triggering global localization reinitialize..."
curl -s -X POST "${API_BASE}/apps/amcl/operations/reinitialize_global_localization/executions" \
  -H "Content-Type: application/json" \
  -d '{}'

echo ""
echo "Waiting for particles to scatter..."
sleep 2

# Now try to navigate - with scattered particles, localization uncertainty is high
echo "Sending navigation goal (with high localization uncertainty)..."
curl -s -X POST "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" \
  -H "Content-Type: application/json" \
  -d '{
    "goal": {
      "pose": {
        "header": {"frame_id": "map"},
        "pose": {
          "position": {"x": 2.0, "y": 0.0, "z": 0.0},
          "orientation": {"w": 1.0}
        }
      }
    }
  }' | jq '.' 2>/dev/null || true

echo ""
echo "✓ Localization failure injected!"
echo ""
echo "AMCL has been reinitialized - localization uncertainty is high."
echo "The anomaly_detector monitors AMCL covariance and will report:"
echo "  - LOCALIZATION_UNCERTAINTY: High AMCL covariance (uncertainty)"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
