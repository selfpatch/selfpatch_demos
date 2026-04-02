#!/bin/bash
# Smoke tests for multi_ecu_aggregation demo
# Runs from the host against the perception ECU gateway on localhost:8080
# The perception ECU is the primary aggregator - it should expose local entities
# plus aggregated entities from planning and actuation ECUs.
#
# Usage: ./tests/smoke_test_multi_ecu.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
# shellcheck disable=SC2034  # Used by smoke_lib.sh
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

# --- Wait for gateway startup ---

# Multi-ECU demo needs extra time: 3 containers building their ROS graphs
# plus aggregation discovery across the Docker network
wait_for_gateway 120

# Wait for runtime node linking (perception ECU local nodes)
wait_for_runtime_linking "/apps/lidar-driver/data" 90

# Wait for aggregation to discover peer ECUs
echo "  Waiting for aggregated entities from planning ECU (max 60s)..."
if poll_until "/apps" '.items[] | select(.id == "path-planner")' 60; then
    echo -e "  ${GREEN}Aggregation discovery complete${NC}"
else
    echo -e "  ${RED}Aggregation discovery did not complete within 60s${NC}"
    exit 1
fi

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

section "Entity Discovery - Components"

# The aggregator should see: robot-alpha (shared parent), perception-ecu, planning-ecu, actuation-ecu
if api_get "/components"; then
    for comp_id in robot-alpha perception-ecu planning-ecu actuation-ecu; do
        if echo "$RESPONSE" | items_contain_id "$comp_id"; then
            pass "components contains '${comp_id}'"
        else
            fail "components contains '${comp_id}'" "not found in response"
        fi
    done
else
    fail "GET /components returns 200" "unexpected status code"
fi

section "Entity Discovery - Apps"

# 10 demo apps across 3 ECUs:
#   Perception: lidar-driver, camera-driver, point-cloud-filter, object-detector
#   Planning:   path-planner, behavior-planner, task-scheduler
#   Actuation:  motor-controller, joint-driver, gripper-controller
if api_get "/apps"; then
    for app_id in \
        lidar-driver camera-driver point-cloud-filter object-detector \
        path-planner behavior-planner task-scheduler \
        motor-controller joint-driver gripper-controller; do
        if echo "$RESPONSE" | items_contain_id "$app_id"; then
            pass "apps contains '${app_id}'"
        else
            fail "apps contains '${app_id}'" "not found in response"
        fi
    done
    # Verify at least 10 demo apps are present
    local_count=$(echo "$RESPONSE" | jq '[.items[] | select(.id | test("^(lidar|camera|point|object|path|behavior|task|motor|joint|gripper)"))] | length')
    if [ "$local_count" -ge 10 ]; then
        pass "at least 10 demo apps discovered"
    else
        fail "at least 10 demo apps discovered" "found ${local_count}"
    fi
else
    fail "GET /apps returns 200" "unexpected status code"
fi

section "Entity Discovery - Functions"

# 3 cross-ECU functions: autonomous-navigation, obstacle-avoidance, task-execution
if api_get "/functions"; then
    for func_id in autonomous-navigation obstacle-avoidance task-execution; do
        if echo "$RESPONSE" | items_contain_id "$func_id"; then
            pass "functions contains '${func_id}'"
        else
            fail "functions contains '${func_id}'" "not found in response"
        fi
    done
else
    fail "GET /functions returns 200" "unexpected status code"
fi

section "Data Access"

# Test data access on a local perception app
assert_non_empty_items "/apps/lidar-driver/data"

section "Configurations"

# Test configurations on a local perception app
assert_non_empty_items "/apps/lidar-driver/configurations"

section "Faults"

if api_get "/faults"; then
    pass "GET /faults returns 200"
else
    fail "GET /faults returns 200" "unexpected status code"
fi

section "Scripts"

# Perception ECU scripts should include inject-sensor-failure
assert_scripts_list "perception-ecu" "inject-sensor-failure"

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
