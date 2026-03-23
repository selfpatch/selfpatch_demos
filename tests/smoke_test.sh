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

test_entity_discovery "areas" sensors processing diagnostics
test_entity_discovery "components" lidar-unit imu-unit gps-unit camera-unit
test_entity_discovery "apps" lidar-sim imu-sim gps-sim camera-sim anomaly-detector

section "Data Access"

assert_non_empty_items "/apps/lidar-sim/data"

section "Configurations"

assert_non_empty_items "/apps/lidar-sim/configurations"

if echo "$RESPONSE" | jq -e '.items[] | select(.name == "noise_stddev")' > /dev/null 2>&1; then
    pass "configurations contains 'noise_stddev' parameter"
else
    fail "configurations contains 'noise_stddev' parameter" "not found in response"
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

# Create a trigger on diagnostic_bridge faults
echo "  Creating OnChange fault trigger..."
TRIGGER_RESPONSE=$(curl -s -w "\n%{http_code}" -X POST "${API_BASE}/apps/diagnostic_bridge/triggers" \
    -H "Content-Type: application/json" \
    -d '{"resource":"/api/v1/apps/diagnostic_bridge/faults","trigger_condition":{"condition_type":"OnChange"},"multishot":true,"lifetime":60}' 2>/dev/null) || true

TRIGGER_HTTP=$(echo "$TRIGGER_RESPONSE" | tail -1)
TRIGGER_BODY=$(echo "$TRIGGER_RESPONSE" | sed '$d')

if [ "$TRIGGER_HTTP" = "201" ]; then
    pass "POST /apps/diagnostic_bridge/triggers returns 201"
else
    fail "POST /apps/diagnostic_bridge/triggers returns 201" "got HTTP $TRIGGER_HTTP"
fi

TRIGGER_ID=$(echo "$TRIGGER_BODY" | jq -r '.id')
if [ -n "$TRIGGER_ID" ] && [ "$TRIGGER_ID" != "null" ]; then
    pass "trigger response contains valid id"
else
    fail "trigger response contains valid id" "id is null or empty"
fi

TRIGGER_STATUS=$(echo "$TRIGGER_BODY" | jq -r '.status')
if [ "$TRIGGER_STATUS" = "active" ]; then
    pass "trigger status is 'active'"
else
    fail "trigger status is 'active'" "got '$TRIGGER_STATUS'"
fi

# List triggers - verify it appears
if api_get "/apps/diagnostic_bridge/triggers"; then
    if echo "$RESPONSE" | jq -e --arg id "$TRIGGER_ID" '.items[] | select(.id == $id)' > /dev/null 2>&1; then
        pass "GET /apps/diagnostic_bridge/triggers lists created trigger"
    else
        fail "GET /apps/diagnostic_bridge/triggers lists created trigger" "trigger $TRIGGER_ID not found"
    fi
else
    fail "GET /apps/diagnostic_bridge/triggers returns 200" "unexpected status code"
fi

# Delete trigger
TRIGGER_DELETE_STATUS=$(curl -s -o /dev/null -w "%{http_code}" -X DELETE \
    "${API_BASE}/apps/diagnostic_bridge/triggers/${TRIGGER_ID}" 2>/dev/null) || true

if [ "$TRIGGER_DELETE_STATUS" = "204" ]; then
    pass "DELETE /apps/diagnostic_bridge/triggers/$TRIGGER_ID returns 204"
else
    fail "DELETE /apps/diagnostic_bridge/triggers/$TRIGGER_ID returns 204" "got HTTP $TRIGGER_DELETE_STATUS"
fi

# Verify trigger is gone
if api_get "/apps/diagnostic_bridge/triggers"; then
    if ! echo "$RESPONSE" | jq -e --arg id "$TRIGGER_ID" '.items[] | select(.id == $id)' > /dev/null 2>&1; then
        pass "trigger no longer listed after deletion"
    else
        fail "trigger no longer listed after deletion" "still found in list"
    fi
else
    fail "GET /apps/diagnostic_bridge/triggers returns 200 after delete" "unexpected status code"
fi

# --- Summary ---

print_summary
