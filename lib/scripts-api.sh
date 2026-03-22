#!/bin/bash
# Shared helper for calling gateway Scripts API
# Source this from host-side wrapper scripts:
#   SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
#   source "${SCRIPT_DIR}/../../lib/scripts-api.sh"
#
# Environment variables:
#   GATEWAY_URL    - Gateway base URL (default: http://localhost:8080)
#   POLL_INTERVAL  - Seconds between status polls (default: 1)
#   MAX_WAIT       - Max seconds to wait for completion (default: 120)

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"
POLL_INTERVAL="${POLL_INTERVAL:-1}"
MAX_WAIT="${MAX_WAIT:-120}"

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

# Execute a script via Scripts API and poll until completion
# Usage: execute_script <entity_type> <entity_id> <script_id> [description]
# entity_type: "components" or "apps"
execute_script() {
    local entity_type="$1"
    local entity_id="$2"
    local script_id="$3"
    local description="${4:-$script_id}"

    check_gateway

    echo "Executing ${description} via Scripts API..."
    local result
    if ! result=$(curl -sf -X POST "${API_BASE}/${entity_type}/${entity_id}/scripts/${script_id}/executions" \
        -H "Content-Type: application/json" \
        -d '{"execution_type": "now"}' 2>/dev/null); then
        echo "Failed to start script execution."
        echo "Check that the script '${script_id}' exists:"
        echo "  curl ${API_BASE}/${entity_type}/${entity_id}/scripts | jq"
        exit 1
    fi

    local exec_id
    exec_id=$(echo "$result" | jq -r '.id')
    if [ -z "$exec_id" ] || [ "$exec_id" = "null" ]; then
        echo "Failed to start script execution:"
        echo "$result" | jq '.' 2>/dev/null || echo "$result"
        exit 1
    fi
    if [[ ! "$exec_id" =~ ^[a-zA-Z0-9_-]+$ ]]; then
        echo "Unexpected execution ID format: $exec_id"
        exit 1
    fi

    echo "Execution started: $exec_id"

    # Poll until done
    local start=$SECONDS
    while [ $((SECONDS - start)) -lt "$MAX_WAIT" ]; do
        local exec_data
        exec_data=$(curl -sf "${API_BASE}/${entity_type}/${entity_id}/scripts/${script_id}/executions/${exec_id}" 2>/dev/null) || exec_data=""

        if [ -z "$exec_data" ]; then
            # Transient failure - keep polling
            printf "." >&2
            sleep "$POLL_INTERVAL"
            continue
        fi

        local status
        status=$(echo "$exec_data" | jq -r '.status')

        case "$status" in
            completed)
                echo ""
                # Show script output if available
                local stdout
                stdout=$(echo "$exec_data" | jq -r '.parameters.stdout // empty' 2>/dev/null)
                if [ -n "$stdout" ]; then
                    echo "$stdout"
                fi
                echo "Done."
                return 0
                ;;
            failed|terminated)
                echo ""
                echo "Script ${status}!"
                echo "$exec_data" | jq -r '.error // empty' 2>/dev/null
                local err_stdout
                err_stdout=$(echo "$exec_data" | jq -r '.parameters.stdout // empty' 2>/dev/null)
                if [ -n "$err_stdout" ]; then
                    echo "$err_stdout"
                fi
                return 1
                ;;
            *)
                printf "." >&2
                sleep "$POLL_INTERVAL"
                ;;
        esac
    done

    echo ""
    echo "Timeout waiting for script execution (${MAX_WAIT}s)"
    return 1
}

# List available scripts for an entity
# Usage: list_scripts <entity_type> <entity_id>
list_scripts() {
    local entity_type="$1"
    local entity_id="$2"

    check_gateway

    curl -sf "${API_BASE}/${entity_type}/${entity_id}/scripts" | jq '.items[] | {id, name, description}' 2>/dev/null
}
