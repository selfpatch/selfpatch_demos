#!/bin/bash
# Smoke tests for turtlebot3_integration navigation in headless mode
#
# Usage (from the repository root):
#   (cd demos/turtlebot3_integration && docker compose --profile ci up -d --build turtlebot3-demo-ci)
#   ./tests/smoke_test_navigation.sh
#
# What this pins, and why each assertion is here:
#
#   1. Every Nav2 lifecycle node reaches active. The simulator is started
#      through a launch argument that has to arrive as separate tokens; when it
#      arrives as one glued token the world never runs, no sensor data reaches
#      ROS, the global costmap cannot get its transform, planner_server hangs in
#      Activating and the lifecycle manager aborts the whole bringup.
#   2. A goal is accepted and completes. This is what fails when the map origin
#      does not match the map, because the robot then stands outside the global
#      costmap and every plan is refused before it starts.
#   3. Localization is not badly wrong. This is a guard against AMCL losing the
#      robot altogether, not the check that pins the initial pose: a configured
#      pose that does not match the spawn point is caught by the goals above,
#      which stop completing. Measured with the pose deliberately put back to
#      (0, 0), the goals fail while this check still passes.
#   4. The inject command setup-triggers.sh hints delivers its fault to the
#      event stream watch-triggers.sh reads.
#   5. After that inject restore-normal.sh alone restores the demo: AMCL agrees
#      with the simulated pose, localization is not reported again and goals
#      complete, so the script can run again on the same stack.
#
# The existing turtlebot3 smoke test deliberately does not navigate, which is
# why the first three could ship together unnoticed.

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

DEMO_CONTAINER="${DEMO_CONTAINER:-turtlebot3_medkit_demo_ci}"

NAV_APP="bt-navigator"
LIFECYCLE_APPS=(bt-navigator planner-server controller-server amcl)

# The stack either activates together or aborts the bringup, so the first node to
# answer sets the pace and the rest follow within seconds. These budgets are
# generous against a loaded runner without letting one wedged node hold the CI
# job for a quarter of an hour.
LIFECYCLE_TIMEOUT=120
GOAL_TIMEOUT=120
# Faults are reported asynchronously, so give the detector a window to deliver
# one before concluding the drive was clean.
FAULT_SETTLE=5

# The robot spawns at (-2.0, -0.5). Both goals are short moves in free space, so
# a failure means the navigation stack is broken rather than the goal being
# unreachable. There are two of them, and the second returns to the spawn point,
# because a single goal leaves the robot standing on it: the goal checker's
# xy_goal_tolerance is 0.25 m, so a second run against the same container would
# report success without the robot moving at all.
SPAWN_X=-2.0
SPAWN_Y=-0.5
GOAL_A_X=-1.5
GOAL_A_Y=-0.5
GOAL_B_X="$SPAWN_X"
GOAL_B_Y="$SPAWN_Y"

# The detector that reports localization quality, and the node name it must
# register under for its reports to mean anything.
DETECTOR_APP="anomaly-detector"
DETECTOR_NODE="/bridge/anomaly_detector"

# --- Helpers ---
#
# smoke_lib.sh sets `pipefail`, and `grep -q` closes the pipe on its first
# match, so a long producer such as `docker logs` dies of SIGPIPE and the
# pipeline reports failure even though the pattern was found. Matches below read
# a captured string with a here-string instead.

# Read a lifecycle node's current state through its SOVD operation.
# Usage: lifecycle_state APP
lifecycle_state() {
    local app="$1" body label
    body=$(curl -s -m 20 -X POST \
        "${API_BASE}/apps/${app}/operations/get_state/executions" \
        -H 'Content-Type: application/json' -d '{"parameters":{}}' 2>/dev/null) || true
    # A body cut short by a timeout is not JSON; jq then prints nothing and
    # exits non-zero, and both cases mean the state is unavailable.
    label=$(jq -r '.parameters.current_state.label // "unavailable"' <<< "$body" 2>/dev/null) || true
    echo "${label:-unavailable}"
}

# Wait for a lifecycle node to report a state, then assert it.
# Usage: assert_lifecycle_active APP [max_wait]
assert_lifecycle_active() {
    local app="$1" max_wait="${2:-$LIFECYCLE_TIMEOUT}" elapsed=0 state=""
    while [ "$elapsed" -lt "$max_wait" ]; do
        state=$(lifecycle_state "$app")
        if [ "$state" = "active" ]; then
            break
        fi
        sleep 5
        elapsed=$((elapsed + 5))
    done
    if [ "$state" = "active" ]; then
        pass "${app} reaches lifecycle state active"
    else
        fail "${app} reaches lifecycle state active" \
             "state is '${state}' after ${elapsed}s"
    fi
}

# Assert a pattern does not appear in the demo container's log.
# Usage: refute_in_container_log PATTERN DESCRIPTION
refute_in_container_log() {
    local pattern="$1" description="$2" logs
    logs=$(docker logs "$DEMO_CONTAINER" 2>&1) || {
        fail "$description" "could not read logs of ${DEMO_CONTAINER}"
        return
    }
    # An empty log would satisfy any refutation, so require the container to
    # have said something first.
    if [ -z "$logs" ]; then
        fail "$description" "container log is empty, nothing was measured"
        return
    fi
    if grep -q "$pattern" <<< "$logs"; then
        fail "$description" "found '${pattern}' in the container log"
    else
        pass "$description"
    fi
}

# Report whether localization is badly wrong: AMCL's position spread above the
# detector's ERROR threshold, the level at which it raises LOCALIZATION_UNCERTAINTY
# at ERROR severity.
#
# The WARN level is deliberately not enough. AMCL's particle spread widens while
# the robot drives and settles again afterwards, and on a healthy run of this
# demo it touches 0.307 against a warn threshold of 0.300 - a two percent
# crossing. Failing on that would turn the demo doing its job into a red build,
# while a real pose error, the kind this test exists to catch, is metres wide and
# clears the error threshold by a wide margin.
#
# The spread is read from AMCL, not from the fault list: a fault keeps the
# highest severity it ever had, even across a clear, so after one ERROR every
# later WARN reads ERROR.
#
# Echoes "yes", "no", or "error". The error case matters: a failed or malformed
# read must not read as "no fault", or this passes whenever the gateway is
# unreachable.
localization_badly_wrong() {
    local threshold spread
    threshold=$(curl -s -m 20 \
        "${API_BASE}/apps/${DETECTOR_APP}/configurations/covariance_error_threshold" \
        | jq -e '.data | numbers' 2>/dev/null) || threshold=""
    spread=$(curl -s -m 20 "${API_BASE}/apps/amcl/data/amcl_pose" \
        | jq -e '.data.pose.covariance | (.[0] + .[7]) | sqrt' 2>/dev/null) || spread=""
    if [ -z "$threshold" ] || [ -z "$spread" ]; then
        echo error
        return
    fi
    jq -nr --argjson s "$spread" --argjson t "$threshold" 'if $s > $t then "yes" else "no" end'
}

# Usage: assert_localization_certain WHEN
assert_localization_certain() {
    local when="$1" state
    state=$(localization_badly_wrong)
    case "$state" in
        no)  pass "localization is not badly wrong ${when}" ;;
        yes) fail "localization is not badly wrong ${when}" "AMCL position spread is above the detector's ERROR threshold" ;;
        *)   fail "localization is not badly wrong ${when}" "could not read the AMCL pose or the threshold" ;;
    esac
}

# Drive to a pose and require the goal to be accepted and to finish.
# Usage: drive_to X Y [WHEN]
drive_to() {
    local goal_x="$1" goal_y="$2" when="${3:+ $3}" body http_code payload execution_id status elapsed poll_body
    payload=$(jq -nc --argjson x "$goal_x" --argjson y "$goal_y" \
        '{parameters: {pose: {header: {frame_id: "map"},
          pose: {position: {x: $x, y: $y, z: 0.0},
                 orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
          behavior_tree: ""}}')
    body=$(curl -s -m 60 -w "\n%{http_code}" -X POST \
        "${API_BASE}/apps/${NAV_APP}/operations/navigate_to_pose/executions" \
        -H 'Content-Type: application/json' -d "$payload" 2>/dev/null) || true
    http_code=$(tail -1 <<< "$body")
    body=$(sed '$d' <<< "$body")

    if [ "$http_code" = "202" ]; then
        pass "goal (${goal_x}, ${goal_y}) is accepted${when}"
    else
        fail "goal (${goal_x}, ${goal_y}) is accepted${when}" \
             "HTTP ${http_code}: $(head -c 200 <<< "$body")"
        return
    fi

    execution_id=$(jq -r '.id // empty' <<< "$body" 2>/dev/null) || true
    if [ -z "$execution_id" ]; then
        fail "goal (${goal_x}, ${goal_y}) completes${when}" "no execution id in the accept response"
        return
    fi

    status="running"
    elapsed=0
    while [ "$elapsed" -lt "$GOAL_TIMEOUT" ]; do
        # A read that does not come back leaves the status unavailable, which
        # the case below reports as a lost goal. jq prints nothing for an empty
        # body and its // fallback never runs, so the fallback is set here.
        poll_body=$(curl -s -m 20 \
            "${API_BASE}/apps/${NAV_APP}/operations/navigate_to_pose/executions/${execution_id}" \
            2>/dev/null) || true
        status=$(jq -r '.status // "unavailable"' <<< "$poll_body" 2>/dev/null) || true
        [ -n "$status" ] || status="unavailable"
        case "$status" in
            completed|succeeded|failed) break ;;
        esac
        sleep 5
        elapsed=$((elapsed + 5))
    done

    case "$status" in
        completed|succeeded) pass "goal (${goal_x}, ${goal_y}) completes${when}" ;;
        *) fail "goal (${goal_x}, ${goal_y}) completes${when}" "status is '${status}' after ${elapsed}s" ;;
    esac
}

# --- Preconditions ---

wait_for_gateway 240

section "Fault reporting is alive"

# Without this, an empty fault list below would be indistinguishable from a
# detector that never started, and the check that LOCALIZATION_UNCERTAINTY
# stays absent would pass by saying nothing.
if poll_until "/apps/${DETECTOR_APP}" \
    ".[\"x-medkit\"].ros2.node == \"${DETECTOR_NODE}\"" 120; then
    pass "detector app reports ROS node ${DETECTOR_NODE}"
else
    fail "detector app reports ROS node ${DETECTOR_NODE}" \
         "got $(jq -c '.["x-medkit"].ros2 // "no x-medkit.ros2"' <<< "$RESPONSE" 2>/dev/null)"
fi

section "Nav2 lifecycle"

for app in "${LIFECYCLE_APPS[@]}"; do
    assert_lifecycle_active "$app"
done

refute_in_container_log "Aborting bringup" "lifecycle manager brings up every Nav2 node"

section "Costmap covers the robot"

# The planner refuses before it starts when the robot is outside the costmap,
# and the costmap says so on every update, so this fires long before a goal is
# ever sent.
refute_in_container_log "out of bounds of the costmap" "robot is inside the global costmap"
refute_in_container_log "out of map bounds" "sensor origin is inside the map"

# A fault confirmed before the drive would otherwise be blamed on the drive. The
# default profile confirms on one event and never heals, so one uncertain moment
# at startup would fail this test on every later run against the same container.
assert_localization_certain "before the drive"

section "Navigation goal"

drive_to "$GOAL_A_X" "$GOAL_A_Y"
drive_to "$GOAL_B_X" "$GOAL_B_Y"

section "Localization held while driving"

sleep "$FAULT_SETTLE"
assert_localization_certain "after the drive"

section "Trigger delivers fault events"

# Drive setup-triggers.sh and watch-triggers.sh exactly as a user would, and run
# the inject command setup-triggers.sh prints, so a hint that stops producing a
# delivered event fails here.
TB3_DIR="${SCRIPT_DIR}/../demos/turtlebot3_integration"
INJECTED_CODE="LOCALIZATION_UNCERTAINTY"

SETUP_OUTPUT=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash ./setup-triggers.sh 2>&1) || true
TRIGGER_ID=$(sed -n 's/^  ID:[[:space:]]*//p' <<< "$SETUP_OUTPUT" | head -1)
INJECT_CMD=$(sed -n '/^Then inject a fault in another terminal:$/{n;s/^  //p;}' <<< "$SETUP_OUTPUT")

if [ -n "$TRIGGER_ID" ]; then
    pass "setup-triggers.sh creates a trigger on apps/${DETECTOR_APP}"
else
    fail "setup-triggers.sh creates a trigger on apps/${DETECTOR_APP}" "$(tail -5 <<< "$SETUP_OUTPUT")"
fi

# Prints the fault code of every complete event watch-triggers.sh has logged.
# Usage: event_fault_codes LOG
event_fault_codes() {
    awk '/Event received:$/ { on = 1; buf = ""; next }
         on && /^---$/ { printf "%s", buf; on = 0; next }
         on { buf = buf $0 "\n" }' "$1" \
        | jq -rs '.[].payload.fault_code // empty' 2>/dev/null || true
}

if [ -n "$TRIGGER_ID" ] && [ -z "$INJECT_CMD" ]; then
    fail "the hinted inject delivers a ${INJECTED_CODE} event to watch-triggers.sh" \
         "setup-triggers.sh printed no inject hint"
elif [ -n "$TRIGGER_ID" ]; then
    WATCH_LOG=$(mktemp)
    # exec makes the PID timeout's own; timeout passes a kill on to the whole stream.
    (cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" exec timeout 60 bash ./watch-triggers.sh "$TRIGGER_ID") \
        > "$WATCH_LOG" 2>&1 &
    WATCH_PID=$!
    sleep 2

    (cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash -c "$INJECT_CMD") > /dev/null 2>&1 || true

    elapsed=0
    while [ "$elapsed" -lt 20 ] && ! grep -qx "$INJECTED_CODE" <<< "$(event_fault_codes "$WATCH_LOG")"; do
        sleep 1
        elapsed=$((elapsed + 1))
    done
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true

    EVENT_CODES=$(event_fault_codes "$WATCH_LOG" | sort -u | paste -sd, -)
    if grep -qx "$INJECTED_CODE" <<< "$(event_fault_codes "$WATCH_LOG")"; then
        pass "the hinted inject delivers a ${INJECTED_CODE} event to watch-triggers.sh"
    else
        fail "the hinted inject delivers a ${INJECTED_CODE} event to watch-triggers.sh" \
             "ran '${INJECT_CMD}', events carried: ${EVENT_CODES:-none}"
    fi
    rm -f "$WATCH_LOG"

    curl -s -o /dev/null -X DELETE "${API_BASE}/apps/${DETECTOR_APP}/triggers/${TRIGGER_ID}" || true
fi

section "Demo restored after the injected fault"

# Echoes {x, y, yaw} of a pose object with position and orientation. Protobuf
# JSON from Gazebo leaves zero fields out, hence the defaults.
POSE_TO_XY_YAW=$(cat <<'JQ'
{x: (.position.x // 0), y: (.position.y // 0),
 yaw: (.orientation | [(.w // 0), (.x // 0), (.y // 0), (.z // 0)] as [$w, $x, $y, $z]
       | atan2(2 * ($w * $z + $x * $y); 1 - 2 * ($y * $y + $z * $z)))}
JQ
)

# Echoes the robot's pose in the simulation as {x, y, yaw}, or nothing. The map
# frame of this demo is the Gazebo world frame.
sim_pose() {
    docker exec "$DEMO_CONTAINER" bash -c \
        'source /opt/ros/jazzy/setup.bash > /dev/null 2>&1
         timeout 10 gz topic -e -n 1 -t /world/default/dynamic_pose/info --json-output 2> /dev/null \
             | jq -c --arg m "$TURTLEBOT3_MODEL" ".pose[] | select(.name == \$m)"' 2> /dev/null \
        | jq -c "$POSE_TO_XY_YAW" 2> /dev/null || true
}

# Echoes AMCL's last published pose as {x, y, yaw}, or nothing.
amcl_pose() {
    curl -s -m 20 "${API_BASE}/apps/amcl/data/amcl_pose" \
        | jq -c ".data.pose.pose | $POSE_TO_XY_YAW" 2> /dev/null || true
}

# Echoes "yes" when LOCALIZATION_UNCERTAINTY is reported as failing at any
# severity, "no" when it is not, and "error" when the list cannot be read.
localization_reported() {
    if ! api_get "/faults?status=all" || ! jq -e '.items' <<< "$RESPONSE" > /dev/null 2>&1; then
        echo error
        return
    fi
    if jq -e '.items[] | select(.fault_code == "LOCALIZATION_UNCERTAINTY"
                                and (.status == "CONFIRMED" or .status == "PREFAILED"))' \
            <<< "$RESPONSE" > /dev/null 2>&1; then
        echo yes
    else
        echo no
    fi
}

# The inject leaves AMCL with a uniform particle cloud. restore-normal.sh alone
# must bring it back to the robot's real pose.
RESTORE_OUTPUT=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash ./restore-normal.sh 2>&1) || true
if grep -q "^Done\.$" <<< "$RESTORE_OUTPUT"; then
    pass "restore-normal.sh completes"
else
    fail "restore-normal.sh completes" "$(tail -5 <<< "$RESTORE_OUTPUT")"
fi

# AMCL publishes a pose only when it updates, and the detector judges a
# published pose at most once every 5 s. An update forced every second for two
# such intervals makes the detector judge the restored localization.
DETECTOR_INTERVAL=5
restored_state="no"
for _ in $(seq 1 $((2 * DETECTOR_INTERVAL))); do
    curl -s -m 10 -o /dev/null -X POST \
        "${API_BASE}/apps/amcl/operations/request_nomotion_update/executions" \
        -H 'Content-Type: application/json' -d '{"parameters":{}}' 2>/dev/null || true
    sleep 1
    restored_state=$(localization_reported)
    [ "$restored_state" = "no" ] || break
done
case "$restored_state" in
    no)  pass "LOCALIZATION_UNCERTAINTY stays absent for $((2 * DETECTOR_INTERVAL))s of forced AMCL updates" ;;
    yes) fail "LOCALIZATION_UNCERTAINTY stays absent for $((2 * DETECTOR_INTERVAL))s of forced AMCL updates" \
              "the detector reported it again" ;;
    *)   fail "LOCALIZATION_UNCERTAINTY stays absent for $((2 * DETECTOR_INTERVAL))s of forced AMCL updates" \
              "could not read the fault list" ;;
esac

# A cloud that converged on the wrong place also stops being reported, so compare
# AMCL's estimate with the robot's pose in the simulation.
POSE_TOLERANCE_M=0.2
POSE_TOLERANCE_RAD=0.2
SIM_POSE=$(sim_pose)
AMCL_POSE=$(amcl_pose)
if [ -z "$SIM_POSE" ] || [ -z "$AMCL_POSE" ]; then
    fail "AMCL agrees with the simulated pose after restore-normal.sh" \
         "could not read a pose: simulation '${SIM_POSE}', AMCL '${AMCL_POSE}'"
elif jq -e -n --argjson s "$SIM_POSE" --argjson a "$AMCL_POSE" \
        --argjson tm "$POSE_TOLERANCE_M" --argjson tr "$POSE_TOLERANCE_RAD" '
        (($s.x - $a.x) * ($s.x - $a.x) + ($s.y - $a.y) * ($s.y - $a.y) | sqrt) <= $tm
        and ((($s.yaw - $a.yaw) | atan2(sin; cos)) | fabs) <= $tr' > /dev/null 2>&1; then
    pass "AMCL agrees with the simulated pose after restore-normal.sh"
else
    fail "AMCL agrees with the simulated pose after restore-normal.sh" \
         "simulation ${SIM_POSE}, AMCL ${AMCL_POSE}, tolerance ${POSE_TOLERANCE_M} m and ${POSE_TOLERANCE_RAD} rad"
fi

# The second goal returns the robot to the spawn point, where the next run
# of this script expects it.
drive_to "$GOAL_A_X" "$GOAL_A_Y" "after the restore"
drive_to "$GOAL_B_X" "$GOAL_B_Y" "after the restore"
sleep "$FAULT_SETTLE"
assert_localization_certain "after the restored drive"

# Clear what the goals above reported, so the next run starts from an empty list.
curl -s -X DELETE "${API_BASE}/faults" > /dev/null || true

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
