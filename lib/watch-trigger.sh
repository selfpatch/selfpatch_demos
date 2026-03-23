#!/bin/bash
# Generic trigger watcher - called by per-demo watch-triggers.sh wrappers
# Requires ENTITY_TYPE and ENTITY_ID to be set before sourcing
set -eu

if [ -z "${ENTITY_TYPE:-}" ] || [ -z "${ENTITY_ID:-}" ]; then
    echo "ENTITY_TYPE and ENTITY_ID must be set before sourcing this script." >&2
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/triggers-api.sh"

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
