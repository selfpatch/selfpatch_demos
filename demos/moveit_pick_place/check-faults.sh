#!/bin/bash
# Check current faults from ros2_medkit gateway
# Faults are collected from MoveIt/Panda via manipulation_monitor

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🔍 Checking faults from ros2_medkit gateway..."
echo ""

# Check for jq dependency
if ! command -v jq >/dev/null 2>&1; then
    echo "❌ 'jq' is required but not installed."
    echo "   Please install jq (e.g., 'sudo apt-get install jq') and retry."
    exit 1
fi

# Wait for gateway
echo "Checking gateway health..."
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    echo "   Start with: ./run-demo.sh"
    exit 1
fi
echo "✓ Gateway is healthy"
echo ""

# Get all faults. A failed read is not "no faults": the gateway answers 503
# while the fault manager is unavailable (for example during startup).
echo "📋 Active Faults:"
RESPONSE=$(curl -s -m 30 -w '\n%{http_code}' "${API_BASE}/faults") || true
HTTP_CODE="${RESPONSE##*$'\n'}"
FAULTS="${RESPONSE%$'\n'*}"

if [ "$HTTP_CODE" != "200" ] || ! FAULT_COUNT=$(echo "$FAULTS" | jq -e '.items | arrays | length' 2>/dev/null); then
    DETAIL=$(echo "$FAULTS" | jq -r '[.message, .parameters.details] | map(select(. != null)) | join(": ")' 2>/dev/null)
    echo "❌ Could not read faults (HTTP ${HTTP_CODE:-000})${DETAIL:+: ${DETAIL}}"
    echo "   The fault list is unknown, not empty. Retry in a few seconds."
    exit 1
fi

if [ "$FAULT_COUNT" = "0" ]; then
    echo "   No active faults — system is healthy! ✅"
else
    echo "$FAULTS" | jq '.items[] | {
        code: .fault_code,
        severity: .severity_label,
        status: .status,
        description: .description,
        sources: .reporting_sources,
        occurrences: .occurrence_count,
        first_occurred: .first_occurred,
        last_occurred: .last_occurred
    }'
fi

echo ""
echo "📊 Fault Summary:"
echo "   Total active faults: $FAULT_COUNT"

# Show fault counts by severity if any exist
if [ "$FAULT_COUNT" != "0" ]; then
    echo ""
    echo "   By severity:"
    echo "$FAULTS" | jq -r '.items | group_by(.severity_label) | .[] | "     \(.[0].severity_label): \(length)"'
fi

echo ""
echo "Commands:"
echo "   Clear all faults: curl -X DELETE ${API_BASE}/faults"
echo "   Check area faults: curl ${API_BASE}/areas/manipulation/faults | jq"
echo "   Check component faults: curl ${API_BASE}/components/panda-arm/faults | jq"
