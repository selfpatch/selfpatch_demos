#!/bin/bash
# Create fault-monitoring trigger for turtlebot3 integration demo
# Alerts on navigation failure faults on the nav2-stack component
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/triggers-api.sh"

ENTITY_TYPE="components"
ENTITY_ID="nav2-stack"

echo "Setting up fault trigger for ${ENTITY_TYPE}/${ENTITY_ID}..."
echo ""

result=$(create_fault_trigger "$ENTITY_TYPE" "$ENTITY_ID")
trigger_id=$(echo "$result" | jq -r '.id')
status=$(echo "$result" | jq -r '.status')
event_source=$(echo "$result" | jq -r '.event_source')

echo "Trigger created successfully!"
echo "  ID:     ${trigger_id}"
echo "  Status: ${status}"
echo "  Events: ${GATEWAY_URL}${event_source}"
echo ""
echo "To watch for events:"
echo "  ./watch-triggers.sh"
echo ""
echo "Then inject a fault in another terminal:"
echo "  ./inject-nav-failure.sh"
