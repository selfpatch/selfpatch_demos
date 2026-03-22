#!/bin/bash
# Smoke tests for moveit_pick_place demo
# Runs from the host against the containerized gateway on localhost:8080
#
# Tests: health, entity discovery (areas, components, apps from manifest)
# Uses demo.launch.py (fake hardware, no Gazebo) for CI stability
#
# Usage: ./tests/smoke_test_moveit.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

# --- Wait for gateway startup ---

wait_for_gateway 120

# Wait for runtime node linking
wait_for_runtime_linking "/apps/medkit-gateway/data" 90

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

test_entity_discovery "areas" manipulation planning diagnostics bridge
test_entity_discovery "components" panda-arm panda-gripper moveit-planning pick-place-loop gateway fault-manager diagnostic-bridge
test_entity_discovery "apps" joint-state-broadcaster panda-arm-controller panda-hand-controller robot-state-publisher move-group pick-place-node medkit-gateway medkit-fault-manager diagnostic-bridge-app manipulation-monitor

section "Logs"

if api_get "/logs"; then
    if echo "$RESPONSE" | jq -e '.items | length > 0' > /dev/null 2>&1; then
        pass "GET /logs returns non-empty items"
    else
        fail "GET /logs returns non-empty items" "items is empty"
    fi
else
    fail "GET /logs returns 200" "unexpected status code"
fi

if api_get "/apps/medkit-gateway/logs"; then
    if echo "$RESPONSE" | jq -e '.items | length > 0' > /dev/null 2>&1; then
        pass "GET /apps/medkit-gateway/logs returns non-empty items"
    else
        fail "GET /apps/medkit-gateway/logs returns non-empty items" "items is empty"
    fi
else
    fail "GET /apps/medkit-gateway/logs returns 200" "unexpected status code"
fi

# --- Summary ---

print_summary
