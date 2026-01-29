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
echo "✓ Localization failure injected!"
echo ""
echo "AMCL will now have high uncertainty until it re-localizes."
echo "Expected faults (via diagnostic_bridge):"
echo "  - AMCL: Localization confidence low"
echo "  - BT_NAVIGATOR: Goal may fail due to uncertain pose"
echo ""
echo "Watch localization recover with:"
echo "  curl ${API_BASE}/apps/amcl/data/particlecloud | jq '.poses | length'"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
