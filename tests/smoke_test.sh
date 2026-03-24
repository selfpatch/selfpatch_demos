#!/bin/bash
# Smoke tests for sensor_diagnostics demo
# Runs from the host against the containerized gateway on localhost:8080
#
# Usage: ./tests/smoke_test.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

# --- Wait for gateway startup ---

wait_for_gateway 90

# Wait for entity discovery + runtime linking (nodes need to be linked to manifest apps)
# In hybrid mode, manifest entities appear instantly but data/configurations require
# the runtime refresh cycle to link ROS 2 nodes to manifest apps.
wait_for_runtime_linking "/apps/lidar-sim/data" 60

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

test_entity_discovery "areas" sensors processing diagnostics bridge
test_entity_discovery "components" lidar-unit imu-unit gps-unit camera-unit compute-unit gateway fault-manager diagnostic-bridge-unit
test_entity_discovery "apps" lidar-sim imu-sim gps-sim camera-sim anomaly-detector medkit-gateway medkit-fault-manager diagnostic-bridge
test_entity_discovery "functions" sensor-monitoring anomaly-detection fault-management

section "Discovery Relationships"

assert_non_empty_items "/areas/sensors/components"

section "Linux Introspection"

assert_procfs_introspection "lidar-sim"

section "Data Access"

assert_non_empty_items "/apps/lidar-sim/data"

section "Configurations"

assert_non_empty_items "/apps/lidar-sim/configurations"

if echo "$RESPONSE" | jq -e '.items[] | select(.name == "noise_stddev")' > /dev/null 2>&1; then
    pass "configurations contains 'noise_stddev' parameter"
else
    fail "configurations contains 'noise_stddev' parameter" "not found in response"
fi

section "Operations"

# fault_manager services may take extra time to be discovered via runtime graph introspection
echo "  Waiting for fault-manager operations to appear (max 30s)..."
if poll_until "/apps/medkit-fault-manager/operations" '.items | length > 0' 30; then
    pass "GET /apps/medkit-fault-manager/operations returns non-empty items"
else
    fail "GET /apps/medkit-fault-manager/operations returns non-empty items" "items still empty after 30s"
fi

section "Scripts"

assert_scripts_list "compute-unit" "run-diagnostics"
assert_script_execution "compute-unit" "run-diagnostics" 30

section "Bulk Data"

# Bulk data endpoint should return 200 with categories list (may be empty without faults)
if api_get "/apps/diagnostic-bridge/bulk-data"; then
    pass "GET /apps/diagnostic-bridge/bulk-data returns 200"
else
    fail "GET /apps/diagnostic-bridge/bulk-data returns 200" "unexpected status code"
fi

section "Logs"

assert_non_empty_items "/apps/medkit-gateway/logs"

section "Fault Injection"

# Inject noise fault via configuration API
echo "  Injecting noise fault (noise_stddev=0.5)..."
INJECT_STATUS=$(curl -s -o /dev/null -w "%{http_code}" \
    -X PUT "${API_BASE}/apps/lidar-sim/configurations/noise_stddev" \
    -H "Content-Type: application/json" \
    -d '{"value": 0.5}') || true

if [ -z "$INJECT_STATUS" ]; then
    fail "PUT noise_stddev=0.5 returns 200" "request failed (no HTTP status received)"
elif [ "$INJECT_STATUS" = "200" ]; then
    pass "PUT noise_stddev=0.5 returns 200"
else
    fail "PUT noise_stddev=0.5 returns 200" "got status $INJECT_STATUS"
fi

# Wait for fault to appear (pipeline: config change -> sensor detects -> /diagnostics -> bridge -> fault_manager)
echo "  Waiting for LIDAR_SIM fault to appear (max 30s)..."
if poll_until "/faults" '.items[] | select(.fault_code == "LIDAR_SIM")' 30; then
    pass "LIDAR_SIM fault appeared in /faults"
else
    fail "LIDAR_SIM fault appeared in /faults" "fault not found after 30s"
fi

# Check fault detail with environment data
if api_get "/apps/diagnostic-bridge/faults/LIDAR_SIM"; then
    pass "GET fault detail returns 200"
    if echo "$RESPONSE" | jq -e '.environment_data' > /dev/null 2>&1; then
        pass "fault detail contains environment_data"
    else
        fail "fault detail contains environment_data" "field missing from response"
    fi
else
    fail "GET fault detail returns 200" "unexpected status code"
fi

# Cleanup: restore config + delete fault
echo "  Cleaning up: restoring config and clearing fault..."
curl -s -X PUT "${API_BASE}/apps/lidar-sim/configurations/noise_stddev" \
    -H "Content-Type: application/json" -d '{"value": 0.01}' > /dev/null || true

curl -s -X DELETE "${API_BASE}/apps/diagnostic-bridge/faults/LIDAR_SIM" > /dev/null || true

# Verify fault is cleared (poll to avoid race with fault manager processing the DELETE)
echo "  Verifying LIDAR_SIM fault cleared (max 5s)..."
elapsed=0
cleared=false
while [ $elapsed -lt 5 ]; do
    if api_get "/faults" && ! echo "$RESPONSE" | jq -e '.items[] | select(.fault_code == "LIDAR_SIM")' > /dev/null 2>&1; then
        pass "LIDAR_SIM fault cleared after cleanup"
        cleared=true
        break
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done
if [ "$cleared" = false ]; then
    fail "LIDAR_SIM fault cleared after cleanup" "fault still present after 5s"
fi

section "Triggers"

assert_triggers_crud "apps" "diagnostic-bridge" "/api/v1/apps/diagnostic-bridge/faults"

section "Beacon Discovery"

# Beacon data is exposed at vendor extension endpoints:
#   /apps/{id}/x-medkit-topic-beacon  (BEACON_MODE=topic)
#   /apps/{id}/x-medkit-param-beacon  (BEACON_MODE=param)
# When BEACON_MODE=none (CI default), these endpoints return 404.
beacon_found=false
for beacon_type in topic-beacon param-beacon; do
    if api_get "/apps/lidar-sim/x-medkit-${beacon_type}"; then
        beacon_found=true
        pass "GET /apps/lidar-sim/x-medkit-${beacon_type} returns 200"
        if echo "$RESPONSE" | jq -e '.status' > /dev/null 2>&1; then
            pass "beacon response contains 'status' field"
        else
            fail "beacon response contains 'status' field" "field missing"
        fi
        if echo "$RESPONSE" | jq -e '.entity_id' > /dev/null 2>&1; then
            pass "beacon response contains 'entity_id' field"
        else
            fail "beacon response contains 'entity_id' field" "field missing"
        fi
        break
    fi
done
if [ "$beacon_found" = false ]; then
    # Not a failure - beacons are optional depending on BEACON_MODE
    echo -e "  ${BLUE}SKIP${NC} beacon not active (BEACON_MODE=none or plugin not loaded)"
fi

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
