#!/bin/bash
# Generic trigger setup - called by per-demo setup-triggers.sh wrappers
# Requires ENTITY_TYPE, ENTITY_ID, and INJECT_HINT to be set before sourcing
set -eu

if [ -z "${ENTITY_TYPE:-}" ] || [ -z "${ENTITY_ID:-}" ]; then
    echo "ENTITY_TYPE and ENTITY_ID must be set before sourcing this script." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/triggers-api.sh"

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
echo "  ${INJECT_HINT:-./inject-nan.sh}"
