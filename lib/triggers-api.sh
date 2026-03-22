#!/bin/bash
# Shared helper for gateway Triggers API
# Source this from host-side trigger scripts:
#   SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
#   source "${SCRIPT_DIR}/../../lib/triggers-api.sh"
#
# Environment variables:
#   GATEWAY_URL    - Gateway base URL (default: http://localhost:8080)

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

# Check dependencies
for cmd in curl jq; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Required tool '$cmd' is not installed."
        exit 1
    fi
done

# Check gateway is reachable
check_gateway() {
    if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
        echo "Gateway not available at ${GATEWAY_URL}. Is the demo running? Try: ./run-demo.sh"
        exit 1
    fi
}

# Create an OnChange trigger on an entity's faults collection
# Usage: create_fault_trigger <entity_type> <entity_id> [lifetime]
# Returns: trigger JSON on stdout
create_fault_trigger() {
    local entity_type="$1"
    local entity_id="$2"
    local lifetime="${3:-3600}"

    check_gateway

    local resource="${API_BASE}/${entity_type}/${entity_id}/faults"
    local body
    body=$(jq -n \
        --arg resource "$resource" \
        --argjson lifetime "$lifetime" \
        '{
            resource: $resource,
            trigger_condition: {condition_type: "OnChange"},
            multishot: true,
            lifetime: $lifetime
        }')

    local result
    local http_code
    result=$(curl -s -w "\n%{http_code}" -X POST \
        "${API_BASE}/${entity_type}/${entity_id}/triggers" \
        -H "Content-Type: application/json" \
        -d "$body" 2>/dev/null) || true

    http_code=$(echo "$result" | tail -1)
    result=$(echo "$result" | sed '$d')

    if [ "$http_code" != "201" ]; then
        echo "Failed to create trigger (HTTP $http_code):" >&2
        echo "$result" | jq '.' 2>/dev/null >&2 || echo "$result" >&2
        return 1
    fi

    echo "$result"
}

# List triggers for an entity
# Usage: list_triggers <entity_type> <entity_id>
list_triggers() {
    local entity_type="$1"
    local entity_id="$2"

    check_gateway

    curl -sf "${API_BASE}/${entity_type}/${entity_id}/triggers" 2>/dev/null | jq '.'
}

# Delete a trigger
# Usage: delete_trigger <entity_type> <entity_id> <trigger_id>
delete_trigger() {
    local entity_type="$1"
    local entity_id="$2"
    local trigger_id="$3"

    check_gateway

    local http_code
    http_code=$(curl -s -o /dev/null -w "%{http_code}" -X DELETE \
        "${API_BASE}/${entity_type}/${entity_id}/triggers/${trigger_id}" 2>/dev/null) || true

    if [ "$http_code" = "204" ]; then
        echo "Trigger ${trigger_id} deleted."
    else
        echo "Failed to delete trigger ${trigger_id} (HTTP $http_code)." >&2
        return 1
    fi
}

# Watch SSE events for a trigger (blocking - Ctrl+C to stop)
# Usage: watch_trigger_events <entity_type> <entity_id> <trigger_id>
watch_trigger_events() {
    local entity_type="$1"
    local entity_id="$2"
    local trigger_id="$3"

    check_gateway

    local url="${API_BASE}/${entity_type}/${entity_id}/triggers/${trigger_id}/events"

    echo "Listening for trigger events..."
    echo "  Entity: ${entity_type}/${entity_id}"
    echo "  Trigger: ${trigger_id}"
    echo "  SSE URL: ${url}"
    echo ""
    echo "Waiting for events (Ctrl+C to stop)..."
    echo "---"

    # Stream SSE events, filter out keepalives, pretty-print JSON data lines
    curl -sf -N "${url}" 2>/dev/null | while IFS= read -r line; do
        # Skip empty lines and keepalive comments
        if [ -z "$line" ] || [[ "$line" == ":"* ]]; then
            continue
        fi
        # Parse SSE data lines
        if [[ "$line" == data:* ]]; then
            local data="${line#data:}"
            # Trim leading space if present
            data="${data# }"
            local timestamp
            timestamp=$(echo "$data" | jq -r '.timestamp // empty' 2>/dev/null)
            if [ -n "$timestamp" ]; then
                echo "[${timestamp}] Event received:"
                echo "$data" | jq '.' 2>/dev/null || echo "$data"
                echo "---"
            else
                echo "$data" | jq '.' 2>/dev/null || echo "$data"
                echo "---"
            fi
        fi
    done
}

# Find the first active trigger for an entity
# Usage: find_active_trigger <entity_type> <entity_id>
# Returns: trigger_id on stdout, or empty string if none found
find_active_trigger() {
    local entity_type="$1"
    local entity_id="$2"

    check_gateway

    local result
    result=$(curl -sf "${API_BASE}/${entity_type}/${entity_id}/triggers" 2>/dev/null) || true

    if [ -z "$result" ]; then
        echo ""
        return 0
    fi

    local tid
    tid=$(echo "$result" | jq -r '.items[] | select(.status == "active") | .id' 2>/dev/null | head -1) || true
    echo "${tid:-}"
}
