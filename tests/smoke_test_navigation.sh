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
#   3. Localization stays certain. AMCL has no RViz here to be told where the
#      robot is, so its configured initial pose has to match the spawn point;
#      when it does not, the covariance stays high and the robot cannot follow a
#      path even though planning succeeds.
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

# The robot spawns at (-2.0, -0.5). This goal is a short move in free space, so
# a failure means the navigation stack is broken rather than the goal being
# unreachable.
GOAL_X=-1.5
GOAL_Y=-0.5

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
    local app="$1" max_wait="${2:-240}" elapsed=0 state=""
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

# --- Preconditions ---

wait_for_gateway 240

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

section "Navigation goal"

GOAL_BODY=$(jq -nc --argjson x "$GOAL_X" --argjson y "$GOAL_Y" \
    '{parameters: {pose: {header: {frame_id: "map"},
      pose: {position: {x: $x, y: $y, z: 0.0},
             orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
      behavior_tree: ""}}')

GOAL_RESPONSE=$(curl -s -m 60 -w "\n%{http_code}" -X POST \
    "${API_BASE}/apps/${NAV_APP}/operations/navigate_to_pose/executions" \
    -H 'Content-Type: application/json' -d "$GOAL_BODY" 2>/dev/null) || true
GOAL_HTTP=$(tail -1 <<< "$GOAL_RESPONSE")
GOAL_BODY_OUT=$(sed '$d' <<< "$GOAL_RESPONSE")

if [ "$GOAL_HTTP" = "202" ]; then
    pass "navigation goal is accepted"
else
    fail "navigation goal is accepted" \
         "HTTP ${GOAL_HTTP}: $(head -c 200 <<< "$GOAL_BODY_OUT")"
fi

EXECUTION_ID=$(jq -r '.id // empty' <<< "$GOAL_BODY_OUT" 2>/dev/null)

if [ -n "$EXECUTION_ID" ]; then
    GOAL_STATUS="running"
    ELAPSED=0
    while [ "$ELAPSED" -lt 180 ]; do
        GOAL_STATUS=$(curl -s -m 20 \
            "${API_BASE}/apps/${NAV_APP}/operations/navigate_to_pose/executions/${EXECUTION_ID}" \
            2>/dev/null | jq -r '.status // "unavailable"' 2>/dev/null)
        case "$GOAL_STATUS" in
            completed|succeeded|failed) break ;;
        esac
        sleep 5
        ELAPSED=$((ELAPSED + 5))
    done
    case "$GOAL_STATUS" in
        completed|succeeded)
            pass "navigation goal completes"
            ;;
        *)
            fail "navigation goal completes" "status is '${GOAL_STATUS}' after ${ELAPSED}s"
            ;;
    esac
else
    fail "navigation goal completes" "no execution id in the accept response"
fi

section "Localization held while driving"

# A confirmed LOCALIZATION_UNCERTAINTY here means AMCL started somewhere the
# robot is not, which is what happens when its configured initial pose does not
# match the spawn point.
if api_get "/faults?status=all"; then
    if jq -e '.items[] | select(.fault_code == "LOCALIZATION_UNCERTAINTY" and .status == "CONFIRMED")' \
        <<< "$RESPONSE" > /dev/null 2>&1; then
        fail "localization stays certain during the drive" \
             "LOCALIZATION_UNCERTAINTY reached CONFIRMED"
    else
        pass "localization stays certain during the drive"
    fi
else
    fail "localization stays certain during the drive" "GET /faults?status=all did not return 200"
fi

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
