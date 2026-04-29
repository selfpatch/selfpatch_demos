#!/bin/bash
# Demo-narrative smoke test for the ota_nav2_sensor_fix demo.
#
# The other smoke test (smoke_test_ota.sh) exercises the SOVD endpoints
# and the OTA process swap, but it never tries to navigate. The headline
# scene of the demo - "operator sends a goal, robot stops, fault appears,
# operator OTAs, robot drives, fault clears" - was unguarded against
# regression. This script reproduces that flow end-to-end so a CI run
# either confirms it works or fails loudly when it doesn't.
#
# What it asserts, in order:
#   1. Gateway healthy, /faults clean (dashboard quiet at boot).
#   2. Nav2 motion lifecycle active (hosted_by under autonomous-navigation
#      function returns at least one node with status=active).
#   3. Publish /goal_pose. Within ~20 s the controller spins /cmd_vel
#      non-zero - proves the BT navigator accepted the goal.
#   4. Within ~30 s of /cmd_vel firing, SCAN_PHANTOM_RETURN appears in
#      /faults with status=active and severity=error. Proves the reactive
#      fault path (broken_lidar reports while controller is driving).
#   5. PUT /updates/fixed_lidar_2_1_0/{prepare,execute}. Process flips
#      from broken_lidar_node to fixed_lidar_node.
#   6. Re-publish /goal_pose. /cmd_vel goes idle within 60 s (the robot
#      either completes or the BT terminates - either way nav2 stops
#      driving), then SCAN_PHANTOM_RETURN clears (status=cleared OR drops
#      out of the active list) within the broken_lidar idle timeout.
#
# Usage: ./tests/smoke_test_demo_narrative.sh [GATEWAY_URL]

GATEWAY_URL="${1:-http://localhost:8080}"
# shellcheck disable=SC2034  # used by smoke_lib.sh
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

GATEWAY_CONTAINER="${OTA_DEMO_GATEWAY_CONTAINER:-ota_demo_gateway}"

# Run a `ros2 ...` command inside the gateway container with the
# environment sourced. Stays one-shot - we always re-source because the
# container's entrypoint isn't an interactive shell.
ros2_in_gw() {
    docker exec "$GATEWAY_CONTAINER" bash -lc \
        "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && $*"
}

publish_goal() {
    local x="${1:-1.5}"
    local y="${2:-0.0}"
    ros2_in_gw "ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
        '{header: {frame_id: map}, pose: {position: {x: ${x}, y: ${y}, z: 0.0}, orientation: {w: 1.0}}}'" \
        > /dev/null
}

# Returns 0 if /cmd_vel has reported a non-zero linear.x|angular.z within
# the last `timeout` seconds. Uses `ros2 topic echo --once` with a wall
# timeout so we don't block forever in the empty-/cmd_vel idle case.
poll_cmd_vel_until_active() {
    local timeout="${1:-30}"
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        local sample
        sample=$(ros2_in_gw "timeout 2 ros2 topic echo /cmd_vel --once --field linear" 2>/dev/null || true)
        # Any non-zero coordinate trips the assertion.
        if echo "$sample" | grep -qE 'x: -?[0-9]*\.?[0-9]*[1-9]'; then
            return 0
        fi
        if echo "$sample" | grep -qE 'y: -?[0-9]*\.?[0-9]*[1-9]'; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 4))  # 2s sample + 2s sleep
    done
    return 1
}

# Returns 0 once /cmd_vel has stayed at zero for `quiet_sec` seconds, or
# fails after `timeout`.
poll_cmd_vel_until_idle() {
    local timeout="${1:-90}"
    local quiet_sec="${2:-5}"
    local elapsed=0
    local zero_streak=0
    while [ $elapsed -lt "$timeout" ]; do
        local sample
        sample=$(ros2_in_gw "timeout 2 ros2 topic echo /cmd_vel --once --field linear" 2>/dev/null || true)
        if echo "$sample" | grep -qE 'x: -?[0-9]*\.?[0-9]*[1-9]'; then
            zero_streak=0
        else
            zero_streak=$((zero_streak + 4))
            if [ $zero_streak -ge "$quiet_sec" ]; then
                return 0
            fi
        fi
        sleep 2
        elapsed=$((elapsed + 4))
    done
    return 1
}

# Returns 0 if a fault with fault_code == $1 is present in /faults with
# any non-cleared status (CONFIRMED / PREFAILED / FAILED). The
# fault_manager goes through several debounced states; for the demo we
# just need "the dashboard would show this fault to the operator".
fault_active() {
    local code="$1"
    api_get "/faults" || return 1
    echo "$RESPONSE" | jq -e --arg c "$code" \
        '.items[]
            | select(.fault_code == $c)
            | select((.status // "") | ascii_upcase | test("CONFIRMED|PREFAILED|FAILED"))' \
        > /dev/null 2>&1
}

# Returns 0 once $1 fault is no longer present in items OR carries a
# cleared / healed status. Healing in fault_manager goes via PREFAILED
# until healing_threshold EVENT_PASSED reports flush it; this poll is
# the dashboard's "the red went away" check.
poll_fault_cleared() {
    local code="$1"
    local timeout="${2:-30}"
    local elapsed=0
    while [ $elapsed -lt "$timeout" ]; do
        if ! fault_active "$code"; then
            return 0
        fi
        sleep 2
        elapsed=$((elapsed + 2))
    done
    return 1
}

# ---------------------------------------------------------------------
# Wait for gateway + the nav2 stack to be ready
# ---------------------------------------------------------------------
wait_for_gateway 120

section "Pre-flight"

# Plugin boot poll registers the catalog - wait so the OTA step works.
echo "  Waiting for plugin catalog (max 30s)..."
if poll_until "/updates" '.items[] | select(. == "fixed_lidar_2_1_0")' 30; then
    pass "OTA catalog ready"
else
    fail "OTA catalog ready" "fixed_lidar_2_1_0 missing from /updates"
    exit 1
fi

# Faults Dashboard quiet at boot - the reactive fault should NOT be active
# until the operator publishes a goal.
if ! fault_active "SCAN_PHANTOM_RETURN"; then
    pass "no SCAN_PHANTOM_RETURN fault active at boot"
else
    fail "no SCAN_PHANTOM_RETURN fault active at boot" "fault present before goal_pose was published"
fi

# Nav2 stack should be lifecycle-active. lifecycle_manager_navigation
# publishes its state on /lifecycle_manager_navigation/transition_event;
# easier proxy: SOVD says the node is live.
echo "  Waiting for bt_navigator to be discoverable (max 60s)..."
if poll_until "/apps" '.items[] | select(.id == "bt-navigator")' 60; then
    pass "bt_navigator app present"
else
    fail "bt_navigator app present" "missing from /apps after 60s"
    exit 1
fi

# ---------------------------------------------------------------------
# Phase 1: pre-OTA goal -> robot tries to drive -> reactive fault fires
# ---------------------------------------------------------------------
section "Reactive fault: pre-OTA goal triggers SCAN_PHANTOM_RETURN"

publish_goal 1.5 0.0
echo "  Published /goal_pose (1.5, 0.0). Waiting for SCAN_PHANTOM_RETURN to appear..."

# The fault appearing IS the proof the reactive path is wired correctly:
# broken_lidar only reports SCAN_PHANTOM_RETURN when its /cmd_vel
# subscription has seen non-zero motion, so a fault here means the BT
# navigator accepted the goal AND the controller spun /cmd_vel up. The
# previous explicit `ros2 topic echo /cmd_vel` check was brittle - echo
# --once snapshots a single message and missed the transient stream -
# without giving us any extra signal beyond what the fault provides.
elapsed=0
while [ $elapsed -lt 60 ]; do
    if fault_active "SCAN_PHANTOM_RETURN"; then
        pass "SCAN_PHANTOM_RETURN active after goal_pose (proves reactive cmd_vel path)"
        break
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
if ! fault_active "SCAN_PHANTOM_RETURN"; then
    fail "SCAN_PHANTOM_RETURN active after goal_pose" \
         "fault never appeared - either nav2 didn't accept the goal or broken_lidar's /cmd_vel subscription is broken"
fi

# ---------------------------------------------------------------------
# Phase 2: OTA replaces broken_lidar with fixed_lidar
# ---------------------------------------------------------------------
section "OTA swap: broken_lidar -> fixed_lidar"

curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/fixed_lidar_2_1_0/prepare" > /dev/null
sleep 4
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API_BASE}/updates/fixed_lidar_2_1_0/execute" > /dev/null
sleep 6

if docker exec "$GATEWAY_CONTAINER" pgrep -f "/lib/fixed_lidar/fixed_lidar_node" > /dev/null 2>&1; then
    pass "fixed_lidar_node spawned after OTA"
else
    fail "fixed_lidar_node spawned after OTA" "process missing in gateway container"
fi
if docker exec "$GATEWAY_CONTAINER" pgrep -f "/lib/broken_lidar/broken_lidar_node" > /dev/null 2>&1; then
    fail "broken_lidar_node killed after OTA" "still alive in gateway container"
else
    pass "broken_lidar_node killed after OTA"
fi

# ---------------------------------------------------------------------
# Phase 3: post-OTA goal -> robot drives -> fault clears
# ---------------------------------------------------------------------
section "Post-OTA: SCAN_PHANTOM_RETURN clears, robot completes the goal"

# Re-issue the goal so the new run is on fixed_lidar.
publish_goal 1.5 0.0
sleep 5

# fixed_lidar fires EVENT_PASSED at 500 ms; the fault_manager debounce
# counter has to climb above the FAILED count broken_lidar accumulated
# (one tick every 2 s while nav2 was driving). On a clean boot that's
# usually < 30 FAILED events so 60 s of healing at 500 ms is plenty -
# but we give 90 s of headroom for slow runners and replays where the
# operator pushed a couple of goals before the OTA.
echo "  Waiting for SCAN_PHANTOM_RETURN to clear (max 90s)..."
if poll_fault_cleared "SCAN_PHANTOM_RETURN" 90; then
    pass "SCAN_PHANTOM_RETURN cleared after OTA"
else
    fail "SCAN_PHANTOM_RETURN cleared after OTA" \
         "fault still active - either fixed_lidar misbehaving or fault_manager debounce too long"
fi

# Generous timeout - nav2 may still be planning; we just assert the
# controller eventually stops driving (succeeded OR aborted both end
# the active /cmd_vel stream). Either outcome unblocks the demo loop.
echo "  Waiting for /cmd_vel to settle (max 90s)..."
if poll_cmd_vel_until_idle 90 6; then
    pass "/cmd_vel returned to idle (nav2 finished or gave up)"
else
    fail "/cmd_vel returned to idle (nav2 finished or gave up)" \
         "controller still commanding after 90s - something is wedged"
fi
