#!/bin/bash
# Check current faults from ros2_medkit gateway
# Faults are collected from Nav2/TurtleBot3 via anomaly_detector (direct) and
# diagnostic_bridge (legacy /diagnostics path)

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

# Get all faults. A failed read leaves the fault list unknown, not empty.
FAULTS_RESPONSE=$(curl -s -w "\n%{http_code}" "${API_BASE}/faults")
FAULTS_CODE=$(tail -n 1 <<< "$FAULTS_RESPONSE")
FAULTS=$(sed '$d' <<< "$FAULTS_RESPONSE")
if [ "$FAULTS_CODE" != "200" ] || ! echo "$FAULTS" | jq -e '.items | type == "array"' > /dev/null 2>&1; then
    echo "❌ Could not read faults from ${GATEWAY_URL} (HTTP ${FAULTS_CODE}): $(echo "$FAULTS" | jq -r '.message // empty' 2>/dev/null)"
    echo "   The fault list is unknown. Check that the fault manager is running, then retry."
    exit 1
fi

echo "📋 Active Faults:"

# Check if there are any faults
FAULT_COUNT=$(echo "$FAULTS" | jq '.items | length')

if [ "$FAULT_COUNT" = "0" ]; then
    echo "   No active faults - system is healthy!"
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
echo "   Check area faults: curl ${API_BASE}/areas/navigation/faults | jq"
echo "   Check component faults: curl ${API_BASE}/components/nav2-stack/faults | jq"
