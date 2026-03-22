#!/bin/bash
# Watch trigger events for sensor diagnostics demo
# Connects to SSE stream and prints fault events in real time
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/triggers-api.sh"

ENTITY_TYPE="apps"
ENTITY_ID="diagnostic_bridge"

trigger_id="${1:-}"

if [ -z "$trigger_id" ]; then
    # Auto-detect: find first active trigger
    trigger_id=$(find_active_trigger "$ENTITY_TYPE" "$ENTITY_ID")
    if [ -z "$trigger_id" ]; then
        echo "No active triggers found for ${ENTITY_TYPE}/${ENTITY_ID}."
        echo "Create one first: ./setup-triggers.sh"
        exit 1
    fi
    echo "Found active trigger: ${trigger_id}"
    echo ""
fi

watch_trigger_events "$ENTITY_TYPE" "$ENTITY_ID" "$trigger_id"
