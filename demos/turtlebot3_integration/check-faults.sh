#!/bin/bash
# Check current faults from ros2_medkit gateway
# Faults are collected from Nav2/TurtleBot3 via diagnostic_bridge

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🔍 Checking faults from ros2_medkit gateway..."
echo ""

# Wait for gateway
echo "Checking gateway health..."
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    echo "   Start with: ./run-demo.sh"
    exit 1
fi
echo "✓ Gateway is healthy"
echo ""

# Get all faults
echo "📋 Active Faults:"
FAULTS=$(curl -s "${API_BASE}/faults")

# Check if there are any faults
FAULT_COUNT=$(echo "$FAULTS" | jq '.items | length')

if [ "$FAULT_COUNT" = "0" ]; then
    echo "   No active faults - system is healthy!"
else
    echo "$FAULTS" | jq '.items[] | {
        code: .code,
        severity: .severity,
        reporter: .reporter_id,
        message: .message,
        timestamp: .timestamp
    }'
fi

echo ""
echo "📊 Fault Summary:"
echo "   Total active faults: $FAULT_COUNT"
echo ""
echo "Commands:"
echo "   Clear all faults: curl -X DELETE ${API_BASE}/faults"
echo "   Check specific area: curl ${API_BASE}/areas/robot/faults | jq"
