#!/bin/bash
# Smoke tests for the ota_nav2_sensor_fix demo.
# Runs from the host against the gateway on localhost:8080 and asserts:
#   - the gateway loads our ota_update_plugin as the UpdateProvider
#   - the SOVD catalog is registered with the 3 expected entries
#   - the update detail uses spec field names (update_name, no `name`/`version`)
#   - the update flow actually swaps broken_lidar_node for fixed_lidar_node
#     inside the gateway container
#
# Usage: ./tests/smoke_test_ota.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
# shellcheck disable=SC2034  # Used by smoke_lib.sh
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

GATEWAY_CONTAINER="${OTA_DEMO_GATEWAY_CONTAINER:-ota_demo_gateway}"

EXPECTED_IDS=(
    "fixed_lidar_2_1_0"
    "obstacle_classifier_v2_1_0_0"
    "broken_lidar_legacy_remove"
)

# Confirm a process is or is not running inside the gateway container.
# Usage: assert_process_running <pattern> <description>
#        assert_process_gone   <pattern> <description>
assert_process_running() {
    local pattern="$1"
    local desc="$2"
    if docker exec "$GATEWAY_CONTAINER" pgrep -f "$pattern" >/dev/null 2>&1; then
        pass "$desc"
    else
        fail "$desc" "no process matching '$pattern' in $GATEWAY_CONTAINER"
    fi
}

assert_process_gone() {
    local pattern="$1"
    local desc="$2"
    if ! docker exec "$GATEWAY_CONTAINER" pgrep -f "$pattern" >/dev/null 2>&1; then
        pass "$desc"
    else
        fail "$desc" "process matching '$pattern' still alive in $GATEWAY_CONTAINER"
    fi
}

# --- Wait for gateway startup ---

wait_for_gateway 90

# Plugin's boot poll fetches /catalog and registers entries; wait for it.
echo "  Waiting for plugin's boot poll to register catalog (max 30s)..."
if poll_until "/updates" '.items[] | select(. == "fixed_lidar_2_1_0")' 30; then
    echo "  Catalog registered"
else
    echo "  Catalog NOT registered within 30s"
    exit 1
fi

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

section "UpdateProvider plugin loaded"

if docker logs "$GATEWAY_CONTAINER" 2>&1 | grep -q "Update backend provided by plugin"; then
    pass "gateway log says: 'Update backend provided by plugin'"
else
    fail "gateway log says: 'Update backend provided by plugin'" "log line missing"
fi

if docker logs "$GATEWAY_CONTAINER" 2>&1 | grep -q "Updates enabled but no UpdateProvider plugin loaded"; then
    fail "no 'no UpdateProvider' warning" "warning was logged"
else
    pass "no 'no UpdateProvider' warning"
fi

section "Catalog (GET /updates returns SOVD {items})"

if api_get "/updates"; then
    pass "GET /updates returns 200"
else
    fail "GET /updates returns 200" "unexpected status code"
fi

if echo "$RESPONSE" | jq -e '.items | type == "array"' >/dev/null 2>&1; then
    pass "/updates response has items array"
else
    fail "/updates response has items array" "envelope mismatch (SOVD spec violation)"
fi

for id in "${EXPECTED_IDS[@]}"; do
    if echo "$RESPONSE" | jq -e --arg id "$id" '.items[] | select(. == $id)' >/dev/null 2>&1; then
        pass "/updates contains '$id'"
    else
        fail "/updates contains '$id'" "id missing"
    fi
done

section "Detail field shape (SOVD ISO 17978-3 compliance)"

# fixed_lidar update detail: must use spec field names
if api_get "/updates/fixed_lidar_2_1_0"; then
    pass "GET /updates/fixed_lidar_2_1_0 returns 200"

    if echo "$RESPONSE" | jq -e '.update_name' >/dev/null 2>&1; then
        pass "detail has update_name (SOVD spec)"
    else
        fail "detail has update_name (SOVD spec)" "field missing - spec violation"
    fi

    if echo "$RESPONSE" | jq -e '.name' >/dev/null 2>&1; then
        fail "detail does NOT have 'name'" "found 'name' instead of 'update_name'"
    else
        pass "detail does NOT have 'name'"
    fi

    if echo "$RESPONSE" | jq -e '.version' >/dev/null 2>&1; then
        fail "detail does NOT have plain 'version'" "should be x_medkit_version (vendor extension)"
    else
        pass "detail does NOT have plain 'version'"
    fi

    if echo "$RESPONSE" | jq -e '.x_medkit_version == "2.1.0"' >/dev/null 2>&1; then
        pass "detail has x_medkit_version = 2.1.0"
    else
        fail "detail has x_medkit_version = 2.1.0" "field missing or wrong value"
    fi

    if echo "$RESPONSE" | jq -e '.updated_components | index("scan_sensor_node")' >/dev/null 2>&1; then
        pass "detail has updated_components: ['scan_sensor_node']"
    else
        fail "detail has updated_components: ['scan_sensor_node']" "kind metadata missing"
    fi

    if echo "$RESPONSE" | jq -e '.x_medkit_replaces_executable == "broken_lidar_node"' >/dev/null 2>&1; then
        pass "detail has x_medkit_replaces_executable = broken_lidar_node"
    else
        fail "detail has x_medkit_replaces_executable" "field missing"
    fi
fi

section "Initial process state"

assert_process_running "/lib/broken_lidar/broken_lidar_node" "broken_lidar_node running before update"
assert_process_running "broken_lidar_legacy" "broken_lidar_legacy running before uninstall"

section "Update flow: PUT /updates/fixed_lidar_2_1_0/prepare + /execute"

curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/fixed_lidar_2_1_0/prepare" >/dev/null
sleep 4
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/fixed_lidar_2_1_0/execute" >/dev/null
sleep 6

if api_get "/updates/fixed_lidar_2_1_0/status"; then
    if echo "$RESPONSE" | jq -e '.status == "completed"' >/dev/null 2>&1; then
        pass "fixed_lidar_2_1_0 status is completed"
    else
        fail "fixed_lidar_2_1_0 status is completed" "got $(echo "$RESPONSE" | jq -c .)"
    fi
fi

assert_process_gone "/lib/broken_lidar/broken_lidar_node" "broken_lidar_node killed after update"
assert_process_running "/lib/fixed_lidar/fixed_lidar_node" "fixed_lidar_node spawned after update"

section "Install flow: PUT /updates/obstacle_classifier_v2_1_0_0/prepare + /execute"

curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/obstacle_classifier_v2_1_0_0/prepare" >/dev/null
sleep 4
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/obstacle_classifier_v2_1_0_0/execute" >/dev/null
sleep 5

assert_process_running "obstacle_classifier_node" "obstacle_classifier_node spawned after install"
