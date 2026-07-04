#!/bin/bash
# Smoke tests for the ota_nav2_sensor_fix demo.
# Runs from the host against the gateway on localhost:8080 and asserts:
#   - the gateway loads our ota_update_plugin as the UpdateProvider
#   - the boot catalog holds ONLY the bad update (broken_lidar_3_0_0) - the
#     forward hotfix (fixed_lidar_3_0_1) is NOT present until published
#   - ./publish-fix.sh registers fixed_lidar_3_0_1 via SOVD POST /updates,
#     after which it appears in GET /updates
#   - the update detail uses spec field names (update_name, no `name`/`version`)
#   - the publish + apply flow actually swaps broken_lidar_node for
#     fixed_lidar_node inside the gateway container
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

DEMO_DIR="$(cd "${SCRIPT_DIR}/../demos/ota_nav2_sensor_fix" && pwd)"
GATEWAY_CONTAINER="${OTA_DEMO_GATEWAY_CONTAINER:-ota_demo_gateway}"

BOOT_ID="broken_lidar_3_0_0"
FIX_ID="fixed_lidar_3_0_1"

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
if poll_until "/updates" ".items[] | select(. == \"${BOOT_ID}\")" 30; then
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

# Capture logs into a variable and grep via here-string. Piping `printf | grep -q`
# still SIGPIPEs printf when grep -q exits early on first match, and with
# `set -o pipefail` the whole pipeline returns 141 - which `if` reads as
# "no match" even when the line was found. Here-strings avoid the pipe entirely.
GATEWAY_LOGS=$(docker logs "$GATEWAY_CONTAINER" 2>&1 || true)

if grep -q "Update backend provided by plugin" <<<"$GATEWAY_LOGS"; then
    pass "gateway log says: 'Update backend provided by plugin'"
else
    fail "gateway log says: 'Update backend provided by plugin'" "log line missing"
fi

if grep -q "Updates enabled but no UpdateProvider plugin loaded" <<<"$GATEWAY_LOGS"; then
    fail "no 'no UpdateProvider' warning" "warning was logged"
else
    pass "no 'no UpdateProvider' warning"
fi

section "Boot catalog (GET /updates returns SOVD {items}) - bad update only"

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

if echo "$RESPONSE" | jq -e --arg id "$BOOT_ID" '.items[] | select(. == $id)' >/dev/null 2>&1; then
    pass "/updates contains '$BOOT_ID'"
else
    fail "/updates contains '$BOOT_ID'" "id missing"
fi

if echo "$RESPONSE" | jq -e --arg id "$FIX_ID" '.items[] | select(. == $id)' >/dev/null 2>&1; then
    fail "/updates does NOT contain '$FIX_ID' before publish" "the fix leaked into the boot catalog"
else
    pass "/updates does NOT contain '$FIX_ID' before publish"
fi

section "Publish the fix (./publish-fix.sh -> SOVD POST /updates)"

if OTA_GATEWAY_URL="$GATEWAY_URL" OTA_GATEWAY_CONTAINER="$GATEWAY_CONTAINER" \
    "${DEMO_DIR}/publish-fix.sh" >/dev/null; then
    pass "./publish-fix.sh registers ${FIX_ID}"
else
    fail "./publish-fix.sh registers ${FIX_ID}" "publish-fix.sh exited non-zero"
fi

echo "  Waiting for ${FIX_ID} to appear in /updates after publish (max 20s)..."
if poll_until "/updates" ".items[] | select(. == \"${FIX_ID}\")" 20; then
    pass "/updates contains '${FIX_ID}' after publish"
else
    fail "/updates contains '${FIX_ID}' after publish" "id missing after publish-fix.sh"
fi

section "Detail field shape (SOVD ISO 17978-3 compliance)"

# fixed_lidar fix detail: must use spec field names
if api_get "/updates/${FIX_ID}"; then
    pass "GET /updates/${FIX_ID} returns 200"

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

    if echo "$RESPONSE" | jq -e '.x_medkit_version == "3.0.1"' >/dev/null 2>&1; then
        pass "detail has x_medkit_version = 3.0.1"
    else
        fail "detail has x_medkit_version = 3.0.1" "field missing or wrong value"
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

section "/scan SetRemap regression (only broken_lidar publishes, not gz-bridge)"

# config/ros_gz_bridge.yaml bridges the real gz front-laser onto /scan_sim,
# not /scan directly - scan_sensor_node (broken_lidar/fixed_lidar) subscribes
# /scan_sim and republishes onto /scan, leaving it the sole publisher there.
# If that remap regresses, both publishers stomp each other and nav2 sees
# garbage. Use ros2 topic info -v inside the container (host runner has no
# ROS install) and assert exactly one publisher whose node name is NOT
# ros_gz_bridge.
# `ros2 topic info -v` depends on the ros2 daemon's graph cache, which is
# unreliable in some container/DDS setups (it reports 0 publishers for a topic
# that clearly has one, and --no-daemon can hang). Use rclpy's
# get_publishers_info_by_topic instead - it does its own fresh discovery and is
# deterministic. Assert exactly one publisher on /scan and that it is
# scan_sensor_node, not the gz bridge (the gz bridge on /scan would mean the
# /scan_sim remap regressed and both stomp /scan).
# Do the whole check in ONE docker exec and return the verdict via the python
# exit code - no temp file, no captured stdout, no write-then-read race (all of
# which proved flaky under docker-out-of-docker). The probe polls for the
# publisher (scan_sensor_node respawns right after an update swap) and exits 0
# iff /scan has exactly one publisher and it is not the gz bridge.
if docker exec -i "$GATEWAY_CONTAINER" bash -lc \
    'source /opt/ros/jazzy/setup.bash && python3 -' <<'PYEOF'
import rclpy, time, sys
from rclpy.node import Node
rclpy.init()
n = Node('scan_pub_probe')
info = []
for _ in range(15):
    info = n.get_publishers_info_by_topic('/scan')
    if len(info) >= 1:
        break
    time.sleep(1)
names = ' '.join(i.node_name for i in info)
ok = len(info) == 1 and 'ros_gz_bridge' not in names and 'parameter_bridge' not in names
sys.exit(0 if ok else 1)
PYEOF
then
    pass "/scan has exactly 1 publisher (scan_sensor_node, not the gz bridge)"
else
    fail "/scan has exactly 1 publisher (scan_sensor_node, not the gz bridge)" \
        "expected one non-gz-bridge publisher on /scan; SetRemap may have regressed"
fi

section "Apply flow: PUT /updates/${FIX_ID}/prepare + /execute"

curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/${FIX_ID}/prepare" >/dev/null
sleep 4
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/${FIX_ID}/execute" >/dev/null
sleep 6

if api_get "/updates/${FIX_ID}/status"; then
    if echo "$RESPONSE" | jq -e '.status == "completed"' >/dev/null 2>&1; then
        pass "${FIX_ID} status is completed"
    else
        fail "${FIX_ID} status is completed" "got $(echo "$RESPONSE" | jq -c .)"
    fi
fi

assert_process_gone "/lib/broken_lidar/broken_lidar_node" "broken_lidar_node killed after update"
assert_process_running "/lib/fixed_lidar/fixed_lidar_node" "fixed_lidar_node spawned after update"
