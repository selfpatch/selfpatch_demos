#!/bin/bash
# Create fault-monitoring trigger for moveit pick-and-place demo
# Alerts on planning failure faults on the moveit-planning component
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/triggers-api.sh"

ENTITY_TYPE="components"
ENTITY_ID="moveit-planning"

# Check for existing active trigger
existing=$(find_active_trigger "$ENTITY_TYPE" "$ENTITY_ID")
if [ -n "$existing" ]; then
    echo "Active trigger already exists: ${existing}"
    echo "Run ./watch-triggers.sh to connect, or delete it first:"
    echo "  curl -X DELETE ${GATEWAY_URL}/api/v1/${ENTITY_TYPE}/${ENTITY_ID}/triggers/${existing}"
    exit 0
fi

echo "Setting up fault trigger for ${ENTITY_TYPE}/${ENTITY_ID}..."
echo ""

result=$(create_fault_trigger "$ENTITY_TYPE" "$ENTITY_ID")
trigger_id=$(echo "$result" | jq -r '.id')

if [ -z "$trigger_id" ] || [ "$trigger_id" = "null" ]; then
    echo "Failed to parse trigger response." >&2
    echo "$result" >&2
    exit 1
fi

status=$(echo "$result" | jq -r '.status')
event_source=$(echo "$result" | jq -r '.event_source')

echo "Trigger created successfully!"
echo "  ID:     ${trigger_id}"
echo "  Status: ${status}"
echo "  Events: ${GATEWAY_URL}${event_source}"
echo ""
echo "To watch for events:"
echo "  ./watch-triggers.sh ${trigger_id}"
echo ""
echo "Then inject a fault in another terminal:"
echo "  ./inject-planning-failure.sh"
