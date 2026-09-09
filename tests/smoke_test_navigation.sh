#!/bin/bash
# Smoke tests for turtlebot3_integration navigation in headless mode
#
# Usage:
#   cd demos/turtlebot3_integration
#   docker compose --profile ci up -d --build turtlebot3-demo-ci
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
#
# The existing turtlebot3 smoke test deliberately does not navigate, which is
# why all three could ship together unnoticed.

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
GOAL_A_X=-1.5
GOAL_A_Y=-0.5
GOAL_B_X=-2.0
GOAL_B_Y=-0.5

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
    local app="$1" body
    body=$(curl -s -m 20 -X POST \
        "${API_BASE}/apps/${app}/operations/get_state/executions" \
        -H 'Content-Type: application/json' -d '{"parameters":{}}' 2>/dev/null) || true
    jq -r '.parameters.current_state.label // "unavailable"' <<< "$body" 2>/dev/null
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

# Report whether localization is badly wrong: a confirmed LOCALIZATION_UNCERTAINTY
# at ERROR severity, which the detector raises above a covariance of 1.0.
#
# The WARN level is deliberately not enough. AMCL's particle spread widens while
# the robot drives and settles again afterwards, and on a healthy run of this
# demo it touches 0.307 against a warn threshold of 0.300 - a two percent
# crossing. Failing on that would turn the demo doing its job into a red build,
# while a real pose error, the kind this test exists to catch, is metres wide and
# clears the error threshold by a wide margin.
#
# Echoes "yes", "no", or "error". The error case matters: a failed or malformed
# read must not read as "no fault", or this passes whenever the gateway is
# unreachable.
localization_confirmed() {
    if ! api_get "/faults?status=all"; then
        echo error
        return
    fi
    if ! jq -e '.items' <<< "$RESPONSE" > /dev/null 2>&1; then
        echo error
        return
    fi
    if jq -e '.items[] | select(.fault_code == "LOCALIZATION_UNCERTAINTY"
                               and .status == "CONFIRMED"
                               and .severity_label == "ERROR")' \
        <<< "$RESPONSE" > /dev/null 2>&1; then
        echo yes
    else
        echo no
    fi
}

# Usage: assert_localization_certain WHEN
assert_localization_certain() {
    local when="$1" state
    state=$(localization_confirmed)
    case "$state" in
        no)  pass "localization is not badly wrong ${when}" ;;
        yes) fail "localization is not badly wrong ${when}" "LOCALIZATION_UNCERTAINTY is CONFIRMED at ERROR severity" ;;
        *)   fail "localization is not badly wrong ${when}" "could not read the fault list" ;;
    esac
}

# Drive to a pose and require the goal to be accepted and to finish.
# Usage: drive_to X Y
drive_to() {
    local goal_x="$1" goal_y="$2" body http_code payload execution_id status elapsed
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
        pass "goal (${goal_x}, ${goal_y}) is accepted"
    else
        fail "goal (${goal_x}, ${goal_y}) is accepted" \
             "HTTP ${http_code}: $(head -c 200 <<< "$body")"
        return
    fi

    execution_id=$(jq -r '.id // empty' <<< "$body" 2>/dev/null)
    if [ -z "$execution_id" ]; then
        fail "goal (${goal_x}, ${goal_y}) completes" "no execution id in the accept response"
        return
    fi

    status="running"
    elapsed=0
    while [ "$elapsed" -lt "$GOAL_TIMEOUT" ]; do
        status=$(curl -s -m 20 \
            "${API_BASE}/apps/${NAV_APP}/operations/navigate_to_pose/executions/${execution_id}" \
            2>/dev/null | jq -r '.status // "unavailable"' 2>/dev/null)
        case "$status" in
            completed|succeeded|failed) break ;;
        esac
        sleep 5
        elapsed=$((elapsed + 5))
    done

    case "$status" in
        completed|succeeded) pass "goal (${goal_x}, ${goal_y}) completes" ;;
        *) fail "goal (${goal_x}, ${goal_y}) completes" "status is '${status}' after ${elapsed}s" ;;
    esac
}

# --- Preconditions ---

wait_for_gateway 240

section "Fault reporting is alive"

# Without this, an empty fault list below would be indistinguishable from a
# detector that never started, and the localization checks would pass by saying
# nothing.
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

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
