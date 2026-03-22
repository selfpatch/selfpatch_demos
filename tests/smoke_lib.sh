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
    local label
    label="${entity_type^}"

    section "Entity Discovery - ${label}"

    if api_get "/${entity_type}"; then
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

# Print test summary and exit with appropriate code
print_summary() {
    echo ""
    echo -e "${BLUE}================================${NC}"
    local total=$((PASS_COUNT + FAIL_COUNT))
    echo -e "  Results: ${GREEN}${PASS_COUNT} passed${NC}, ${RED}${FAIL_COUNT} failed${NC} (${total} total)"

    if [ "$FAIL_COUNT" -gt 0 ]; then
        echo -e "\n  ${RED}Failed tests:${FAILED_TESTS}${NC}"
        echo -e "${BLUE}================================${NC}"
        exit 1
    fi

    echo -e "${BLUE}================================${NC}"
    echo -e "\n${GREEN}All smoke tests passed!${NC}"
}
