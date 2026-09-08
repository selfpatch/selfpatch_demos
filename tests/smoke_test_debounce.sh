#!/bin/bash
# Smoke tests for the turtlebot3_integration debounce profile
#
# Runs from the host against the containerized gateway, with the demo started
# under the debounce overlay:
#
#   cd demos/turtlebot3_integration
#   docker compose --profile ci -f docker-compose.yml -f docker-compose.debounce.yml up -d --build
#   ./tests/smoke_test_debounce.sh
#
# What this pins:
#   1. The node name the per-source threshold file keys on is the name the
#      detector actually registers. The key is an anchored prefix of the
#      reported source_id, so a namespace change in the launch file makes it
#      stop matching silently: the resolver falls back to the global thresholds
#      and the goal-status faults sink into PREFAILED, where the default fault
#      list never shows them.
#   2. The thresholds themselves. The goal-status source confirms on one FAILED
#      and heals on one PASSED, while the base source still needs three FAILED.
#      That contrast is the whole point of the debounce profile.
#   3. That a confirmed fault on the base source can still heal. Its counter is
#      clamped at the confirmation threshold, so healing costs a burst of PASSED
#      events, not one - which is why the detector answers a recovery with a
#      burst and why healing_threshold is 0.
#
# Faults are injected through the gateway's own SOVD operation endpoint for
# /fault_manager/report_fault rather than by driving Gazebo. Navigation-driven
# injection is too timing-dependent for CI (see the header of
# smoke_test_turtlebot3.sh), and the contract under test is how the fault
# manager resolves thresholds per source, which a report exercises exactly as
# the detector does.

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

DEMO_CONTAINER="${DEMO_CONTAINER:-turtlebot3_medkit_demo_ci}"

# Manifest entity ids for the two nodes this test talks about.
DETECTOR_APP="anomaly-detector"
FAULT_MANAGER_APP="medkit-fault-manager"

# The launch file puts the detector in the "bridge" namespace. This name is
# duplicated in config/entity_thresholds_debounce.yaml (as the key prefix) and
# in scripts/anomaly_detector.py (as the reported source_id); this test is what
# keeps the three in step.
DETECTOR_NODE="/bridge/anomaly_detector"
GOAL_SOURCE="${DETECTOR_NODE}/goal_status"
BASE_SOURCE="${DETECTOR_NODE}"

# Codes are unique per run so a re-run against a live container starts from no
# stored state, and so nothing collides with the codes the real detector reports
# while the test is in flight.
RUN_TAG="$(date +%s)"
GOAL_CODE="DEBOUNCE_SMOKE_GOAL_${RUN_TAG}"
BASE_CODE="DEBOUNCE_SMOKE_BASE_${RUN_TAG}"

EVENT_FAILED=0
EVENT_PASSED=1
SEVERITY_INFO=0
SEVERITY_WARN=1

# --- Helpers ---
#
# smoke_lib.sh sets `pipefail`, and `grep -q` closes the pipe on its first
# match. A producer with more output than a pipe buffer then dies of SIGPIPE and
# the whole pipeline reports failure even though the pattern WAS found - so a
# match on a long `docker logs` reads as a miss while identical code passes on
# short output. Every match below therefore reads a captured string with a
# here-string, never a pipeline.

# Report one fault event the way the detector does: through the SOVD operation
# the gateway exposes for the fault manager's report_fault service.
# Usage: report_fault CODE EVENT_TYPE SEVERITY SOURCE_ID
report_fault() {
    local code="$1" event="$2" severity="$3" source_id="$4"
    local body http_code payload
    payload=$(jq -nc --arg c "$code" --argjson e "$event" --argjson s "$severity" --arg src "$source_id" \
        '{parameters: {fault_code: $c, event_type: $e, severity: $s, description: "debounce smoke", source_id: $src}}')
    body=$(curl -s -w "\n%{http_code}" -X POST \
        "${API_BASE}/apps/${FAULT_MANAGER_APP}/operations/report_fault/executions" \
        -H 'Content-Type: application/json' -d "$payload" 2>/dev/null) || true
    http_code=$(tail -1 <<< "$body")
    body=$(sed '$d' <<< "$body")
    if [ "$http_code" != "200" ]; then
        echo "       report_fault ${code} event=${event} src=${source_id}: HTTP ${http_code}" >&2
        return 1
    fi
    # A 200 that did not accept the report would leave every later assertion
    # measuring a fault that was never recorded.
    if ! jq -e '.parameters.accepted == true' <<< "$body" > /dev/null 2>&1; then
        echo "       report_fault ${code} not accepted: $(head -c 200 <<< "$body")" >&2
        return 1
    fi
    return 0
}

# Report an event and register a pass/fail for the reporting itself.
# Usage: report_or_fail CODE EVENT_TYPE SEVERITY SOURCE_ID DESCRIPTION
report_or_fail() {
    if report_fault "$1" "$2" "$3" "$4"; then
        pass "$5"
    else
        fail "$5" "report_fault call did not succeed"
    fi
}

# Assert a fault reaches a raw debounce status within max_wait seconds.
# Usage: assert_status CODE EXPECTED_STATUS DESCRIPTION [max_wait]
assert_status() {
    local code="$1" expected="$2" description="$3" max_wait="${4:-15}"
    if poll_until "/faults?status=all" \
        ".items[] | select(.fault_code == \"${code}\" and .status == \"${expected}\")" \
        "$max_wait"; then
        pass "$description"
    else
        local got
        got=$(jq -r ".items[] | select(.fault_code == \"${code}\") | .status" <<< "$RESPONSE" 2>/dev/null)
        fail "$description" "status is '${got:-<fault absent>}', expected '${expected}'"
    fi
}

# Assert a fault does NOT hold a status, giving it time to prove it would.
# Usage: refute_status CODE FORBIDDEN_STATUS DESCRIPTION [settle_seconds]
refute_status() {
    local code="$1" forbidden="$2" description="$3" settle="${4:-5}"
    sleep "$settle"
    # An unreachable endpoint must not read as "the status was not reached" -
    # that turns every outage into a silent pass for this whole class of check.
    if ! api_get "/faults?status=all"; then
        fail "$description" "GET /faults?status=all did not return 200"
        return
    fi
    if jq -e ".items[] | select(.fault_code == \"${code}\" and .status == \"${forbidden}\")" <<< "$RESPONSE" > /dev/null 2>&1; then
        fail "$description" "status reached '${forbidden}'"
    elif ! jq -e ".items[] | select(.fault_code == \"${code}\")" <<< "$RESPONSE" > /dev/null 2>&1; then
        # The fault vanishing entirely is not the same as it holding a different
        # status, and would make the refutation vacuous.
        fail "$description" "fault ${code} is absent from the list"
    else
        pass "$description"
    fi
}

# --- Preconditions ---

wait_for_gateway 180

section "Debounce profile is the one that loaded"

# A compose overlay that skips this service starts the demo on the default
# profile, and every threshold assertion below would then measure the wrong
# configuration while still looking plausible. The resolver announces what it
# read at startup, so check for that line rather than for the absence of an
# error - an absent error is also what a container that never got that far
# produces.
LOG_WAIT=0
THRESHOLD_LOADED=""
while [ "$LOG_WAIT" -lt 60 ]; do
    CONTAINER_LOGS=$(docker logs "$DEMO_CONTAINER" 2>&1 || true)
    if grep -q "Loaded 1 entity threshold entries" <<< "$CONTAINER_LOGS"; then
        THRESHOLD_LOADED=yes
        break
    fi
    sleep 3
    LOG_WAIT=$((LOG_WAIT + 3))
done

if [ -n "$THRESHOLD_LOADED" ]; then
    pass "fault manager loaded the per-source threshold file"
else
    fail "fault manager loaded the per-source threshold file" \
         "no 'Loaded 1 entity threshold entries' in logs of ${DEMO_CONTAINER} after ${LOG_WAIT}s"
fi

section "Detector node name matches the threshold key"

# The threshold key is an anchored prefix of the source_id the detector reports,
# and the source_id starts with the node's fully qualified name. The gateway
# publishes that name, so this catches a launch-file rename before the
# behavioural assertions below turn into a confusing threshold failure.
if poll_until "/apps/${DETECTOR_APP}" \
    ".[\"x-medkit\"].ros2.node == \"${DETECTOR_NODE}\"" 120; then
    pass "detector app reports ROS node ${DETECTOR_NODE}"
else
    fail "detector app reports ROS node ${DETECTOR_NODE}" \
         "got $(jq -c '.["x-medkit"].ros2 // "no x-medkit.ros2"' <<< "$RESPONSE" 2>/dev/null)"
fi

section "Fault reporting operation is available"

if poll_until "/apps/${FAULT_MANAGER_APP}/operations" \
    ".items[] | select(.id == \"report_fault\")" 120; then
    pass "report_fault is exposed as an operation on ${FAULT_MANAGER_APP}"
else
    fail "report_fault is exposed as an operation on ${FAULT_MANAGER_APP}" \
         "operation not found; faults cannot be injected"
    exit 1
fi

# --- Goal-status source: confirms on one event, heals on one ---

section "Goal-status source (confirmation_threshold -1, healing_threshold 0)"

report_or_fail "$GOAL_CODE" "$EVENT_FAILED" "$SEVERITY_WARN" "$GOAL_SOURCE" \
    "reported one FAILED as ${GOAL_SOURCE}"

assert_status "$GOAL_CODE" "CONFIRMED" "one FAILED confirms the goal-status fault"

if api_get "/faults?status=all" && \
   jq -e --arg src "$GOAL_SOURCE" \
     ".items[] | select(.fault_code == \"${GOAL_CODE}\") | .reporting_sources | index(\$src)" <<< "$RESPONSE" > /dev/null 2>&1; then
    pass "fault records ${GOAL_SOURCE} as its reporting source"
else
    fail "fault records ${GOAL_SOURCE} as its reporting source" \
         "got $(jq -c ".items[] | select(.fault_code == \"${GOAL_CODE}\") | .reporting_sources" <<< "$RESPONSE" 2>/dev/null)"
fi

report_or_fail "$GOAL_CODE" "$EVENT_PASSED" "$SEVERITY_INFO" "$GOAL_SOURCE" \
    "reported one PASSED as ${GOAL_SOURCE}"

assert_status "$GOAL_CODE" "HEALED" "one PASSED heals the goal-status fault"

# --- Base source: still filtered by the global -3 ---

section "Base source (global confirmation_threshold -3)"

report_or_fail "$BASE_CODE" "$EVENT_FAILED" "$SEVERITY_WARN" "$BASE_SOURCE" \
    "reported one FAILED as ${BASE_SOURCE}"

assert_status "$BASE_CODE" "PREFAILED" "one FAILED leaves the base fault PREFAILED"
refute_status "$BASE_CODE" "CONFIRMED" "one FAILED does not confirm the base fault"

for _ in 1 2; do
    report_or_fail "$BASE_CODE" "$EVENT_FAILED" "$SEVERITY_WARN" "$BASE_SOURCE" \
        "reported a further FAILED as ${BASE_SOURCE}"
done

assert_status "$BASE_CODE" "CONFIRMED" "three FAILED confirm the base fault"

# --- Base source: a confirmed fault still heals, but costs a burst ---

section "Healing a confirmed base fault"

report_or_fail "$BASE_CODE" "$EVENT_PASSED" "$SEVERITY_INFO" "$BASE_SOURCE" \
    "reported one PASSED as ${BASE_SOURCE}"
refute_status "$BASE_CODE" "HEALED" "one PASSED does not heal a confirmed base fault"

for _ in 1 2; do
    report_or_fail "$BASE_CODE" "$EVENT_PASSED" "$SEVERITY_INFO" "$BASE_SOURCE" \
        "reported a further PASSED as ${BASE_SOURCE}"
done

assert_status "$BASE_CODE" "HEALED" "three PASSED heal the confirmed base fault"

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
