#!/bin/bash
# Demo-narrative smoke test for the ota_nav2_sensor_fix demo.
#
# The other smoke test (smoke_test_ota.sh) exercises the SOVD /updates
# endpoints directly; this one drives the demo the way an operator would -
# through the operator scripts (send-goal.sh / publish-fix.sh / apply-fix.sh /
# clear-fault.sh) - and asserts the latch/publish/apply/clear loop end-to-end.
#
# The narrative: the entrypoint auto-applies broken_lidar_3_0_0 before the
# mission starts (a routine fleet update that regressed the lidar). The
# operator sends a goal, the robot drives into the phantom sector, and Nav2
# genuinely cannot make progress - navigate_to_pose aborts. Two generic
# ros2_medkit bridges (not a custom fault in the scan node) turn that Nav2
# failure into SOVD faults: ACTION_NAVIGATE_TO_POSE_ABORTED on bt-navigator
# (the headline fault, with the freeze-frame + rosbag snapshot under its
# environment_data.snapshots) and a content-hashed LOG_CONTROLLER_SERVER_*
# on controller-server (supporting, checked by entity since the hash isn't
# stable). The fix (fixed_lidar_3_0_1) is a forward hotfix - not a rollback
# to a previous build - and is held out of the boot catalog: the operator
# publishes it with ./publish-fix.sh (SOVD POST /updates), then applies it
# with ./apply-fix.sh (prepare/execute) - but both faults are latched (no
# self-heal): they stay CONFIRMED until the operator explicitly clears them.
# Only after the deliberate clear does a fresh goal resume clean.
#
# What it asserts, in order (poll-with-timeout, real HTTP/process checks -
# no hollow asserts). It deliberately does NOT assert full nav2 goal
# completion (flaky) - only the reactive fault/update/process behavior:
#   1. Boot: broken_lidar_3_0_0 is applied (entrypoint auto-apply) and
#      scan_sensor_node is running broken_lidar_node; fixed_lidar_3_0_1 is
#      NOT yet registered (boot catalog holds only the bad update).
#   2. send-goal.sh -> ACTION_NAVIGATE_TO_POSE_ABORTED reaches CONFIRMED on
#      bt-navigator, and controller-server picks up a supporting LOG_* fault.
#   3. Fault detail (bt-navigator) has environment_data.snapshots >= 1, and
#      the rosbag bulk-data download returns a non-empty MCAP body.
#   4. publish-fix.sh -> fixed_lidar_3_0_1 appears in /updates (SOVD
#      POST /updates).
#   5. apply-fix.sh -> scan_sensor_node swaps to fixed_lidar_node, but both
#      faults stay latched (the key regression guard vs the old
#      self-healing behavior).
#   6. clear-fault.sh -> ACTION_NAVIGATE_TO_POSE_ABORTED is gone from
#      bt-navigator and the LOG_* fault is gone from controller-server.
#   7. send-goal.sh again -> neither fault reappears (clean lidar, healthy
#      resume).
#
# Usage: ./tests/smoke_test_demo_narrative.sh [GATEWAY_URL]
# GATEWAY_URL defaults to OTA_GATEWAY_URL, or http://localhost:$OTA_GATEWAY_PORT
# (default port 8080) - the same env vars the operator scripts honor.

GATEWAY_URL="${1:-${OTA_GATEWAY_URL:-http://localhost:${OTA_GATEWAY_PORT:-8080}}}"
# shellcheck disable=SC2034  # used by smoke_lib.sh
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

DEMO_DIR="$(cd "${SCRIPT_DIR}/../demos/ota_nav2_sensor_fix" && pwd)"
GATEWAY_CONTAINER="${OTA_DEMO_GATEWAY_CONTAINER:-ota_demo_gateway}"

BOOT_ID="broken_lidar_3_0_0"
FIX_ID="fixed_lidar_3_0_1"

NAV_ENTITY="apps/bt-navigator"
NAV_CODE="ACTION_NAVIGATE_TO_POSE_ABORTED"
CONTROLLER_ENTITY="apps/controller-server"
# controller-server's LOG_CONTROLLER_SERVER_* code is content-hashed (derived
# from the log message), so it is never matched by exact code - only by
# "does this entity have any fault at all" (see fault_present with code="").

# --- Helpers built on top of smoke_lib.sh's api_get/poll_until -------------

# Returns 0 if a fault is present (any status) in the default GET
# /{entity}/faults list. The gateway's default filter already excludes
# cleared/healed faults, so "present" here means pending or confirmed.
# If $code is empty, matches any fault on the entity (used for the
# content-hashed controller-server LOG_* code).
fault_present() {
    local entity="$1"
    local code="$2"
    api_get "/${entity}/faults" || return 1
    if [ -z "$code" ]; then
        echo "$RESPONSE" | jq -e '.items | length > 0' > /dev/null 2>&1
    else
        echo "$RESPONSE" | jq -e --arg c "$code" '.items[] | select(.fault_code == $c)' > /dev/null 2>&1
    fi
}

# Poll until fault_present() is false, up to $3 seconds.
poll_fault_absent() {
    local entity="$1"
    local code="$2"
    local timeout="${3:-30}"
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        if ! fault_present "$entity" "$code"; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 1
}

# Assert the fault stays absent for the *entire* $3-second window - proves
# it does not reappear, rather than just "hasn't yet".
assert_fault_stays_absent() {
    local entity="$1"
    local code="$2"
    local window="${3:-30}"
    local elapsed=0
    while [ $elapsed -lt "$window" ]; do
        if fault_present "$entity" "$code"; then
            return 1
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 0
}

# Assert the fault stays present for the *entire* $3-second window - proves
# latching (no self-heal), rather than just "hasn't cleared yet". This is
# the regression guard against the old self-healing behavior.
assert_fault_stays_present() {
    local entity="$1"
    local code="$2"
    local window="${3:-60}"
    local elapsed=0
    while [ $elapsed -lt "$window" ]; do
        if ! fault_present "$entity" "$code"; then
            return 1
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 0
}

# Poll until `pgrep -af <pattern>` succeeds inside the gateway container
# (process present), up to $2 seconds.
poll_process_running() {
    local pattern="$1"
    local timeout="${2:-20}"
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        if docker exec "$GATEWAY_CONTAINER" pgrep -af "$pattern" > /dev/null 2>&1; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 1
}

# Poll until `pgrep -af <pattern>` fails inside the gateway container
# (process gone), up to $2 seconds.
poll_process_gone() {
    local pattern="$1"
    local timeout="${2:-20}"
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        if ! docker exec "$GATEWAY_CONTAINER" pgrep -af "$pattern" > /dev/null 2>&1; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 1
}

# ---------------------------------------------------------------------
# Wait for the gateway to come up
# ---------------------------------------------------------------------
wait_for_gateway 120

# ---------------------------------------------------------------------
# Step 1: Boot - entrypoint auto-applies broken_lidar_3_0_0; the fix is not
# registered yet
# ---------------------------------------------------------------------
section "Boot: ${BOOT_ID} auto-applied by the entrypoint; ${FIX_ID} not registered yet"

echo "  Waiting for ${BOOT_ID} to appear in /updates (max 120s)..."
if poll_until "/updates" ".items[] | select(. == \"${BOOT_ID}\")" 120; then
    pass "${BOOT_ID} listed in /updates"
else
    fail "${BOOT_ID} listed in /updates" "missing after 120s - entrypoint auto-apply did not register the update"
    exit 1
fi

echo "  Waiting for ${BOOT_ID} status to reach 'completed' (max 60s)..."
if poll_until "/updates/${BOOT_ID}/status" '.status == "completed"' 60; then
    pass "${BOOT_ID} status is 'completed'"
else
    fail "${BOOT_ID} status is 'completed'" "entrypoint auto-apply (prepare+execute) did not complete within 60s"
    exit 1
fi

echo "  Waiting for scan_sensor_node to run broken_lidar_node (max 20s)..."
if poll_process_running "/lib/broken_lidar/broken_lidar_node" 20; then
    pass "scan_sensor_node runs broken_lidar_node at boot"
else
    fail "scan_sensor_node runs broken_lidar_node at boot" "broken_lidar_node process not found in ${GATEWAY_CONTAINER}"
    exit 1
fi

if api_get "/updates" && echo "$RESPONSE" | jq -e --arg id "$FIX_ID" '.items[] | select(. == $id)' >/dev/null 2>&1; then
    fail "${FIX_ID} is NOT registered at boot" "the forward hotfix leaked into the boot catalog"
else
    pass "${FIX_ID} is NOT registered at boot (boot catalog holds only ${BOOT_ID})"
fi

# ---------------------------------------------------------------------
# Step 2: send-goal.sh -> reactive ACTION_NAVIGATE_TO_POSE_ABORTED
# ---------------------------------------------------------------------
section "Reactive fault: send-goal.sh triggers ACTION_NAVIGATE_TO_POSE_ABORTED"

# x=1.8, y=2.3 (frame map) drives straight into the phantom sector so nav2
# reliably stalls - the send-goal.sh script defaults elsewhere are for
# ad-hoc operator use, not this repeatable regression check.
"${DEMO_DIR}/send-goal.sh" 1.8 2.3

echo "  Waiting for ${NAV_CODE} to reach CONFIRMED on ${NAV_ENTITY} (max 60s)..."
if poll_until "/${NAV_ENTITY}/faults" \
    ".items[] | select(.fault_code == \"${NAV_CODE}\") | select((.status // \"\") | ascii_upcase == \"CONFIRMED\")" \
    60; then
    pass "${NAV_CODE} confirmed on ${NAV_ENTITY} after send-goal.sh"
else
    fail "${NAV_CODE} confirmed on ${NAV_ENTITY} after send-goal.sh" \
         "fault never reached CONFIRMED within 60s - either nav2 didn't accept the goal or the action-status bridge is broken"
fi

echo "  Waiting for a supporting LOG_* fault on ${CONTROLLER_ENTITY} (max 60s)..."
if poll_until "/${CONTROLLER_ENTITY}/faults" '.items | length > 0' 60; then
    pass "supporting LOG_* fault present on ${CONTROLLER_ENTITY}"
else
    fail "supporting LOG_* fault present on ${CONTROLLER_ENTITY}" \
         "no fault appeared within 60s - either nav2 didn't stall or the log bridge is broken"
fi

# ---------------------------------------------------------------------
# Step 3: fault detail environment data + MCAP rosbag capture
# ---------------------------------------------------------------------
section "Fault detail: environment_data snapshot + MCAP rosbag capture"

if api_get "/${NAV_ENTITY}/faults/${NAV_CODE}"; then
    pass "GET /${NAV_ENTITY}/faults/${NAV_CODE} returns 200"
    if echo "$RESPONSE" | jq -e '(.environment_data.snapshots // []) | length >= 1' > /dev/null 2>&1; then
        pass "fault detail has >=1 environment_data snapshot"
    else
        fail "fault detail has >=1 environment_data snapshot" \
             "got $(echo "$RESPONSE" | jq -c '.environment_data.snapshots // []' 2>/dev/null)"
    fi
else
    fail "GET /${NAV_ENTITY}/faults/${NAV_CODE} returns 200" "unexpected status code"
fi

# The MCAP rosbag is written ASYNCHRONOUSLY - the ring buffer is flushed on
# confirm, then rosbag.duration_after_sec more seconds are recorded and the bag
# is finalized + registered a few seconds AFTER the fault confirms. So poll for
# the rosbag snapshot to attach and for the bag to be downloadable, rather than
# checking once (which races the write and 404s).
rosbag_snapshot=no
for _ in $(seq 1 20); do
    if api_get "/${NAV_ENTITY}/faults/${NAV_CODE}" && \
       echo "$RESPONSE" | jq -e '[.environment_data.snapshots[]?.type] | index("rosbag")' > /dev/null 2>&1; then
        rosbag_snapshot=yes
        break
    fi
    sleep 2
done
if [ "$rosbag_snapshot" = "yes" ]; then
    pass "fault detail has a rosbag snapshot (MCAP capture attached)"
else
    fail "fault detail has a rosbag snapshot" \
         "no rosbag snapshot after ~40s: $(echo "$RESPONSE" | jq -c '[.environment_data.snapshots[]?.type]' 2>/dev/null)"
fi

# Binary MCAP body - bypass api_get (it reconstructs $RESPONSE via sed/echo,
# which mangles binary content); write straight to a temp file instead. Poll
# the download until served (same async-capture reason as above). Path is
# GET /apps/bt-navigator/bulk-data/rosbags/ACTION_NAVIGATE_TO_POSE_ABORTED.
rosbag_tmp="$(mktemp)"
rosbag_http=000
for _ in $(seq 1 20); do
    rosbag_http=$(curl -s -o "$rosbag_tmp" -w '%{http_code}' \
        "${API_BASE}/${NAV_ENTITY}/bulk-data/rosbags/${NAV_CODE}" 2>/dev/null) || true
    [ "$rosbag_http" = "200" ] && break
    sleep 2
done
rosbag_bytes=$(wc -c < "$rosbag_tmp" 2>/dev/null || echo 0)
rm -f "$rosbag_tmp"

if [ "$rosbag_http" = "200" ]; then
    pass "GET /${NAV_ENTITY}/bulk-data/rosbags/${NAV_CODE} returns 200"
else
    fail "GET /${NAV_ENTITY}/bulk-data/rosbags/${NAV_CODE} returns 200" "got HTTP ${rosbag_http}"
fi

if [ "${rosbag_bytes:-0}" -gt 0 ] 2>/dev/null; then
    pass "MCAP rosbag body is non-empty (${rosbag_bytes} bytes)"
else
    fail "MCAP rosbag body is non-empty" "body was 0 bytes"
fi

# ---------------------------------------------------------------------
# Step 4: publish-fix.sh -> fixed_lidar_3_0_1 registers via SOVD POST /updates
# ---------------------------------------------------------------------
section "Publish: publish-fix.sh registers ${FIX_ID} (SOVD POST /updates)"

if "${DEMO_DIR}/publish-fix.sh" >/dev/null; then
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

# ---------------------------------------------------------------------
# Step 5: apply-fix.sh -> fixed_lidar, both faults stay latched
# ---------------------------------------------------------------------
section "Apply: apply-fix.sh swaps to fixed_lidar, faults stay latched"

"${DEMO_DIR}/apply-fix.sh"

echo "  Waiting for scan_sensor_node to run fixed_lidar_node (max 20s)..."
if poll_process_running "/lib/fixed_lidar/fixed_lidar_node" 20; then
    pass "scan_sensor_node runs fixed_lidar_node after apply"
else
    fail "scan_sensor_node runs fixed_lidar_node after apply" "fixed_lidar_node process not found in ${GATEWAY_CONTAINER}"
fi

echo "  Waiting for broken_lidar_node to be gone (max 20s)..."
if poll_process_gone "/lib/broken_lidar/broken_lidar_node" 20; then
    pass "broken_lidar_node killed after apply"
else
    fail "broken_lidar_node killed after apply" "broken_lidar_node still alive in ${GATEWAY_CONTAINER}"
fi

echo "  Asserting ${NAV_CODE} stays latched on ${NAV_ENTITY} for 60s post-apply (regression guard vs self-heal)..."
if assert_fault_stays_present "$NAV_ENTITY" "$NAV_CODE" 60; then
    pass "${NAV_CODE} still present on ${NAV_ENTITY} after apply (latched, not self-healed)"
else
    fail "${NAV_CODE} still present on ${NAV_ENTITY} after apply (latched, not self-healed)" \
         "fault disappeared on its own after the fix was applied - self-healing regression"
fi

echo "  Asserting the supporting LOG_* fault stays latched on ${CONTROLLER_ENTITY} for 60s post-apply..."
if assert_fault_stays_present "$CONTROLLER_ENTITY" "" 60; then
    pass "LOG_* fault still present on ${CONTROLLER_ENTITY} after apply (latched, not self-healed)"
else
    fail "LOG_* fault still present on ${CONTROLLER_ENTITY} after apply (latched, not self-healed)" \
         "fault disappeared on its own after the fix was applied - self-healing regression"
fi

# ---------------------------------------------------------------------
# Step 6: clear-fault.sh -> operator clear removes both latched faults
# ---------------------------------------------------------------------
section "Operator clear: clear-fault.sh removes the latched faults"

"${DEMO_DIR}/clear-fault.sh"

echo "  Waiting for ${NAV_CODE} to be gone from /${NAV_ENTITY}/faults (max 30s)..."
if poll_fault_absent "$NAV_ENTITY" "$NAV_CODE" 30; then
    pass "${NAV_CODE} gone from ${NAV_ENTITY} after clear-fault.sh"
else
    fail "${NAV_CODE} gone from ${NAV_ENTITY} after clear-fault.sh" "fault still listed 30s after the operator clear"
fi

echo "  Waiting for ${CONTROLLER_ENTITY} faults to clear (max 30s)..."
if poll_fault_absent "$CONTROLLER_ENTITY" "" 30; then
    pass "LOG_* fault gone from ${CONTROLLER_ENTITY} after clear-fault.sh (clear-all)"
else
    fail "LOG_* fault gone from ${CONTROLLER_ENTITY} after clear-fault.sh (clear-all)" \
         "fault still listed 30s after the operator clear-all"
fi

# ---------------------------------------------------------------------
# Step 7: send-goal.sh again -> healthy resume, no relapse
# ---------------------------------------------------------------------
section "Healthy resume: send-goal.sh on the clean lidar does not reintroduce either fault"

"${DEMO_DIR}/send-goal.sh" 1.8 2.3

echo "  Watching for ${NAV_CODE} to stay absent on ${NAV_ENTITY} for 30s (clean lidar, healthy resume)..."
if assert_fault_stays_absent "$NAV_ENTITY" "$NAV_CODE" 30; then
    pass "${NAV_CODE} does not reappear on ${NAV_ENTITY} (healthy resume)"
else
    fail "${NAV_CODE} does not reappear on ${NAV_ENTITY} (healthy resume)" \
         "fault reappeared on the clean lidar - fixed_lidar or fault_manager regression"
fi

echo "  Watching for ${CONTROLLER_ENTITY} to stay clean for 30s (clean lidar, healthy resume)..."
if assert_fault_stays_absent "$CONTROLLER_ENTITY" "" 30; then
    pass "no LOG_* fault reappears on ${CONTROLLER_ENTITY} (healthy resume)"
else
    fail "no LOG_* fault reappears on ${CONTROLLER_ENTITY} (healthy resume)" \
         "fault reappeared on the clean lidar - fixed_lidar or fault_manager regression"
fi
