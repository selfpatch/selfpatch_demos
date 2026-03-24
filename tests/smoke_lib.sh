#!/bin/bash
# Shared test infrastructure for smoke tests
# Source this file from individual smoke test scripts
#
# Required: set GATEWAY_URL before sourcing, or pass as $1 to the test script
# Required: set API_BASE="${GATEWAY_URL}/api/v1"

set -euo pipefail

# --- Dependency preflight ---

for cmd in curl jq; do
    if ! command -v "$cmd" > /dev/null 2>&1; then
        echo "Error: required command '$cmd' not found in PATH" >&2
        exit 1
    fi
done

# --- Test infrastructure ---

PASS_COUNT=0
FAIL_COUNT=0
FAILED_TESTS=""

RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

pass() {
    PASS_COUNT=$((PASS_COUNT + 1))
    echo -e "  ${GREEN}PASS${NC} $1"
}

fail() {
    FAIL_COUNT=$((FAIL_COUNT + 1))
    FAILED_TESTS="${FAILED_TESTS}\\n  - $1"
    echo -e "  ${RED}FAIL${NC} $1"
    if [ -n "${2:-}" ]; then
        echo -e "       ${RED}$2${NC}"
    fi
}

section() {
    echo -e "\n${BLUE}--- $1 ---${NC}"
}

# Helper: GET endpoint, store response in $RESPONSE, check HTTP status
# Usage: api_get "/health" 200
api_get() {
    local endpoint="$1"
    local expected_status="${2:-200}"
    local http_code
    RESPONSE=$(curl -s -w "\n%{http_code}" "${API_BASE}${endpoint}" 2>/dev/null) || true
    http_code=$(echo "$RESPONSE" | tail -1)
    RESPONSE=$(echo "$RESPONSE" | sed '$d')
    if [ "$http_code" != "$expected_status" ]; then
        return 1
    fi
    return 0
}

# Helper: check that JSON array at .items contains an element with .id == value
# Usage: echo "$JSON" | items_contain_id "lidar-sim"
items_contain_id() {
    local id="$1"
    jq -e --arg id "$id" '.items[] | select(.id == $id)' > /dev/null 2>&1
}

# Helper: poll endpoint until jq filter matches (max wait seconds)
# Usage: poll_until "/faults" '.items[] | select(.fault_code == "LIDAR_SIM")' 15
poll_until() {
    local endpoint="$1"
    local jq_filter="$2"
    local max_wait="${3:-15}"
    local elapsed=0

    while [ $elapsed -lt "$max_wait" ]; do
        if api_get "$endpoint" && echo "$RESPONSE" | jq -e "$jq_filter" > /dev/null 2>&1; then
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    return 1
}

# Wait for gateway to become healthy
# Usage: wait_for_gateway [max_wait_seconds]
wait_for_gateway() {
    local max_wait="${1:-90}"
    section "Waiting for gateway"
    echo -e "  Polling ${API_BASE}/health (max ${max_wait}s)..."
    local elapsed=0
    while [ $elapsed -lt "$max_wait" ]; do
        if curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
            echo -e "  ${GREEN}Gateway ready after ${elapsed}s${NC}"
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    echo -e "  ${RED}Gateway failed to start within ${max_wait}s${NC}"
    exit 1
}

# Wait for runtime node linking (manifest entities need runtime refresh to get data)
# Usage: wait_for_runtime_linking "/apps/lidar-sim/data" [max_wait_seconds]
wait_for_runtime_linking() {
    local data_endpoint="$1"
    local max_wait="${2:-60}"
    echo -e "  Waiting for runtime node linking (max ${max_wait}s)..."
    local elapsed=0
    while [ $elapsed -lt "$max_wait" ]; do
        if api_get "$data_endpoint" && echo "$RESPONSE" | jq -e '.items | length > 0' > /dev/null 2>&1; then
            echo -e "  ${GREEN}Runtime linking complete after ${elapsed}s${NC}"
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    echo -e "  ${RED}Runtime node linking did not complete within ${max_wait}s${NC}"
    exit 1
}

# Helper: assert endpoint returns 200 with non-empty .items array
# Usage: assert_non_empty_items "/apps/lidar-sim/data"
assert_non_empty_items() {
    local endpoint="$1"
    if api_get "$endpoint"; then
        if echo "$RESPONSE" | jq -e '.items | length > 0' > /dev/null 2>&1; then
            pass "GET ${endpoint} returns non-empty items"
        else
            fail "GET ${endpoint} returns non-empty items" "items is empty"
        fi
    else
        fail "GET ${endpoint} returns 200" "unexpected status code"
    fi
}

# Test entity discovery for a given entity type
# Usage: test_entity_discovery "areas" "sensors processing diagnostics"
test_entity_discovery() {
    local entity_type="$1"
    shift
    local entity_ids=("$@")
    local expected_count=${#entity_ids[@]}
    local label
    label="${entity_type^}"

    section "Entity Discovery - ${label}"

    if api_get "/${entity_type}"; then
        # Verify exact count - catches duplicate/synthetic entities leaking through
        local actual_count
        actual_count=$(echo "$RESPONSE" | jq '.items | length')
        if [ "$actual_count" = "$expected_count" ]; then
            pass "${entity_type} count is ${expected_count}"
        else
            local actual_ids
            actual_ids=$(echo "$RESPONSE" | jq -r '[.items[].id] | sort | join(", ")')
            fail "${entity_type} count is ${expected_count}" "got ${actual_count}: ${actual_ids}"
        fi
        for entity_id in "${entity_ids[@]}"; do
            if echo "$RESPONSE" | items_contain_id "$entity_id"; then
                pass "${entity_type} contains '${entity_id}'"
            else
                fail "${entity_type} contains '${entity_id}'" "not found in response"
            fi
        done
    else
        fail "GET /${entity_type} returns 200" "unexpected status code"
    fi
}

# Test that procfs introspection data is available for an app
# Usage: assert_procfs_introspection "lidar-sim"
assert_procfs_introspection() {
    local app_id="$1"
    local endpoint="/apps/${app_id}/x-medkit-procfs"
    if api_get "$endpoint"; then
        if echo "$RESPONSE" | jq -e '.pid' > /dev/null 2>&1; then
            pass "GET ${endpoint} returns procfs data with pid"
        else
            fail "GET ${endpoint} returns procfs data with pid" "pid field missing"
        fi
    else
        fail "GET ${endpoint} returns 200" "unexpected status code"
    fi
}

# Test that scripts are listed for a component
# Usage: assert_scripts_list "compute-unit" "run-diagnostics"
assert_scripts_list() {
    local component_id="$1"
    local expected_script="$2"
    local endpoint="/components/${component_id}/scripts"
    if api_get "$endpoint"; then
        if echo "$RESPONSE" | jq -e '.items | length > 0' > /dev/null 2>&1; then
            pass "GET ${endpoint} returns non-empty items"
        else
            fail "GET ${endpoint} returns non-empty items" "items is empty"
        fi
        if echo "$RESPONSE" | jq -e --arg s "$expected_script" '.items[] | select(.id == $s)' > /dev/null 2>&1; then
            pass "scripts contains '${expected_script}'"
        else
            fail "scripts contains '${expected_script}'" "not found in response"
        fi
    else
        fail "GET ${endpoint} returns 200" "unexpected status code"
    fi
}

# Execute a script and verify it completes
# Usage: assert_script_execution "compute-unit" "run-diagnostics" [max_wait]
assert_script_execution() {
    local component_id="$1"
    local script_id="$2"
    local max_wait="${3:-30}"
    local exec_endpoint="/components/${component_id}/scripts/${script_id}/executions"

    # Start execution
    local exec_response
    exec_response=$(curl -s -w "\n%{http_code}" -X POST "${API_BASE}${exec_endpoint}" \
        -H "Content-Type: application/json" \
        -d '{"execution_type": "now"}' 2>/dev/null) || true
    local exec_http
    exec_http=$(echo "$exec_response" | tail -1)
    local exec_body
    exec_body=$(echo "$exec_response" | sed '$d')

    if [ "$exec_http" != "201" ] && [ "$exec_http" != "200" ] && [ "$exec_http" != "202" ]; then
        fail "POST ${exec_endpoint} starts execution" "got HTTP ${exec_http}"
        return
    fi
    pass "POST ${exec_endpoint} starts execution"

    local exec_id
    exec_id=$(echo "$exec_body" | jq -r '.id')
    if [ -z "$exec_id" ] || [ "$exec_id" = "null" ]; then
        fail "script execution returns valid id" "id is null or empty"
        return
    fi
    pass "script execution returns valid id"

    # Poll until completed
    echo "  Waiting for script '${script_id}' to complete (max ${max_wait}s)..."
    local elapsed=0
    while [ $elapsed -lt "$max_wait" ]; do
        if api_get "${exec_endpoint}/${exec_id}"; then
            local status
            status=$(echo "$RESPONSE" | jq -r '.status')
            case "$status" in
                completed)
                    pass "script '${script_id}' completed successfully"
                    return
                    ;;
                failed|terminated)
                    fail "script '${script_id}' completed successfully" "status: ${status}"
                    return
                    ;;
            esac
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
    fail "script '${script_id}' completed successfully" "timed out after ${max_wait}s"
}

# Test trigger CRUD lifecycle on an entity
# Usage: assert_triggers_crud "apps" "diagnostic-bridge" "/api/v1/apps/diagnostic-bridge/faults"
assert_triggers_crud() {
    local entity_type="$1"
    local entity_id="$2"
    local resource_uri="$3"
    local triggers_endpoint="/${entity_type}/${entity_id}/triggers"

    # Create trigger
    echo "  Creating OnChange trigger on ${entity_type}/${entity_id}..."
    local payload
    payload=$(jq -n --arg resource "$resource_uri" \
        '{resource: $resource, trigger_condition: {condition_type: "OnChange"}, multishot: true, lifetime: 60}')
    local create_response
    create_response=$(curl -s -w "\n%{http_code}" -X POST "${API_BASE}${triggers_endpoint}" \
        -H "Content-Type: application/json" \
        -d "$payload" 2>/dev/null) || true

    local create_http
    create_http=$(echo "$create_response" | tail -1)
    local create_body
    create_body=$(echo "$create_response" | sed '$d')

    if [ "$create_http" = "201" ]; then
        pass "POST ${triggers_endpoint} returns 201"
    else
        fail "POST ${triggers_endpoint} returns 201" "got HTTP ${create_http}"
        return
    fi

    local trigger_id
    trigger_id=$(echo "$create_body" | jq -r '.id')
    if [ -n "$trigger_id" ] && [ "$trigger_id" != "null" ]; then
        pass "trigger response contains valid id"
    else
        fail "trigger response contains valid id" "id is null or empty"
        return
    fi

    local trigger_status
    trigger_status=$(echo "$create_body" | jq -r '.status')
    if [ "$trigger_status" = "active" ]; then
        pass "trigger status is 'active'"
    else
        fail "trigger status is 'active'" "got '${trigger_status}'"
    fi

    # List triggers - verify it appears
    if api_get "${triggers_endpoint}"; then
        if echo "$RESPONSE" | jq -e --arg id "$trigger_id" '.items[] | select(.id == $id)' > /dev/null 2>&1; then
            pass "GET ${triggers_endpoint} lists created trigger"
        else
            fail "GET ${triggers_endpoint} lists created trigger" "trigger ${trigger_id} not found"
        fi
    else
        fail "GET ${triggers_endpoint} returns 200" "unexpected status code"
    fi

    # Delete trigger
    local delete_status
    delete_status=$(curl -s -o /dev/null -w "%{http_code}" -X DELETE \
        "${API_BASE}${triggers_endpoint}/${trigger_id}" 2>/dev/null) || true

    if [ "$delete_status" = "204" ]; then
        pass "DELETE trigger ${trigger_id} returns 204"
    else
        fail "DELETE trigger ${trigger_id} returns 204" "got HTTP ${delete_status}"
    fi

    # Verify trigger is gone
    if api_get "${triggers_endpoint}"; then
        if ! echo "$RESPONSE" | jq -e --arg id "$trigger_id" '.items[] | select(.id == $id)' > /dev/null 2>&1; then
            pass "trigger no longer listed after deletion"
        else
            fail "trigger no longer listed after deletion" "still found in list"
        fi
    else
        fail "GET ${triggers_endpoint} returns 200 after delete" "unexpected status code"
    fi
}

# Print test summary (called via EXIT trap - do not call exit here)
SUMMARY_PRINTED=false
print_summary() {
    # Guard against double-printing when called as both trap and explicit call
    if [ "$SUMMARY_PRINTED" = true ]; then
        return
    fi
    SUMMARY_PRINTED=true

    echo ""
    echo -e "${BLUE}================================${NC}"
    local total=$((PASS_COUNT + FAIL_COUNT))
    echo -e "  Results: ${GREEN}${PASS_COUNT} passed${NC}, ${RED}${FAIL_COUNT} failed${NC} (${total} total)"

    if [ "$FAIL_COUNT" -gt 0 ]; then
        echo -e "\n  ${RED}Failed tests:${FAILED_TESTS}${NC}"
        echo -e "${BLUE}================================${NC}"
        return
    fi

    echo -e "${BLUE}================================${NC}"
    echo -e "\n${GREEN}All smoke tests passed!${NC}"
}
