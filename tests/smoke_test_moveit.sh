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
test_entity_discovery "functions" pick-and-place motion-planning gripper-control fault-management

section "Discovery Relationships"

assert_non_empty_items "/areas/manipulation/components"

section "Linux Introspection"

assert_procfs_introspection "medkit-gateway"

section "Data Access"

assert_non_empty_items "/apps/medkit-gateway/data"

section "Operations"

assert_non_empty_items "/apps/medkit-fault-manager/operations"

section "Configurations"

assert_non_empty_items "/apps/medkit-gateway/configurations"

section "Scripts"

assert_scripts_list "moveit-planning" "arm-self-test"
assert_script_execution "moveit-planning" "arm-self-test" 30

section "Bulk Data"

if api_get "/apps/diagnostic-bridge-app/bulk-data"; then
    pass "GET /apps/diagnostic-bridge-app/bulk-data returns 200"
else
    fail "GET /apps/diagnostic-bridge-app/bulk-data returns 200" "unexpected status code"
fi

section "Faults"

if api_get "/faults"; then
    pass "GET /faults returns 200"
else
    fail "GET /faults returns 200" "unexpected status code"
fi

section "Logs"

assert_non_empty_items "/apps/medkit-gateway/logs"

section "Triggers"

assert_triggers_crud "apps" "diagnostic-bridge-app" "/api/v1/apps/diagnostic-bridge-app/faults"

# --- Summary ---

print_summary
