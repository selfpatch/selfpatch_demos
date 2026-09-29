#!/bin/bash
# Smoke tests for turtlebot3_integration demo
# Runs from the host against the containerized gateway on localhost:8080
#
# Tests: health, entity discovery (areas/components/apps/functions),
#   discovery relationships, Linux introspection, data access, operations,
#   configurations, scripts (list + execution), bulk data, faults, logs,
#   trigger CRUD lifecycle, check-entities.sh and check-faults.sh with one
#   injected navigation failure, check-entities.sh with the simulator stopped,
#   the fault trigger across the navigation failure, check-faults.sh while
#   /faults answers 503
#
# Usage: ./tests/smoke_test_turtlebot3.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

DEMO_CONTAINER="${DEMO_CONTAINER:-turtlebot3_medkit_demo_ci}"
# Anchored so it does not match the bash -c wrapper that runs pgrep.
FAULT_MANAGER_PATTERN='^/root/demo_ws/install/ros2_medkit_fault_manager/lib/ros2_medkit_fault_manager/fault_manager_node '

# SIGSTOP on the fault manager makes the gateway's ListFaults call time out,
# so GET /faults answers 503 as it does before the fault manager is up.
stop_fault_manager() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${FAULT_MANAGER_PATTERN}') || exit 1
        kill -STOP \"\${pid}\"
    " > /dev/null 2>&1
}

resume_fault_manager() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${FAULT_MANAGER_PATTERN}') || exit 0
        kill -CONT \"\${pid}\"
    " > /dev/null 2>&1 || true
}

# The Gazebo server steps the simulation. Stopped, it publishes no sensor data
# while the ROS bridge and its /scan publisher stay up.
SIMULATOR_PATTERN='^gz sim '

stop_simulator() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pids=\$(pgrep -f '${SIMULATOR_PATTERN}') || exit 1
        kill -STOP \${pids}
    " > /dev/null 2>&1
}

resume_simulator() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pids=\$(pgrep -f '${SIMULATOR_PATTERN}') || exit 0
        kill -CONT \${pids}
    " > /dev/null 2>&1 || true
}

# print_summary reads the script's exit status from $?, so hand it the status
# saved on entry. set +e: under errexit `(exit rc)` would end the trap before
# print_summary runs.
cleanup_on_exit() {
    local rc=$?
    set +e
    resume_fault_manager
    resume_simulator
    (exit "${rc}")
    print_summary
}
trap cleanup_on_exit EXIT

# --- Wait for gateway startup ---

# Turtlebot3 needs Gazebo + Nav2 - allow extra startup time
wait_for_gateway 120

# Wait for runtime node linking
wait_for_runtime_linking "/apps/medkit-gateway/data" 90

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

test_entity_discovery "areas" robot navigation diagnostics bridge
test_entity_discovery "components" turtlebot3-base lidar-sensor nav2-stack gateway fault-manager diagnostic-bridge-unit
test_entity_discovery "apps" turtlebot3-node robot-state-publisher gazebo amcl bt-navigator controller-server planner-server velocity-smoother medkit-gateway medkit-fault-manager diagnostic-bridge anomaly-detector
test_entity_discovery "functions" autonomous-navigation robot-control fault-management

section "Discovery Relationships"

assert_non_empty_items "/areas/robot/components"

section "Linux Introspection"

assert_procfs_introspection "medkit-gateway"

section "Data Access"

assert_non_empty_items "/apps/medkit-gateway/data"

section "Operations"

# fault_manager services may take extra time to be discovered in Gazebo-heavy demos
echo "  Waiting for fault-manager operations to appear (max 30s)..."
if poll_until "/apps/medkit-fault-manager/operations" '.items | length > 0' 30; then
    pass "GET /apps/medkit-fault-manager/operations returns non-empty items"
else
    fail "GET /apps/medkit-fault-manager/operations returns non-empty items" "items still empty after 30s"
fi

section "Configurations"

assert_non_empty_items "/apps/medkit-gateway/configurations"

section "Scripts"

assert_scripts_list "nav2-stack" "nav-health-check"
assert_script_execution "nav2-stack" "nav-health-check" 30

section "Bulk Data"

if api_get "/apps/diagnostic-bridge/bulk-data"; then
    pass "GET /apps/diagnostic-bridge/bulk-data returns 200"
else
    fail "GET /apps/diagnostic-bridge/bulk-data returns 200" "unexpected status code"
fi

section "Faults"

if api_get "/faults"; then
    pass "GET /faults returns 200"
else
    fail "GET /faults returns 200" "unexpected status code"
fi

section "Logs"

assert_non_empty_items "/apps/medkit-gateway/logs"

section "Check-Entities and Check-Faults Scripts"

TB3_DIR="${SCRIPT_DIR}/../demos/turtlebot3_integration"

# Echoes a Nav2 node's lifecycle state label, or "unavailable".
# Usage: lifecycle_label APP
lifecycle_label() {
    curl -s -m 20 -X POST "${API_BASE}/apps/$1/operations/get_state/executions" \
        -H 'Content-Type: application/json' -d '{"parameters":{}}' 2>/dev/null \
        | jq -r '.parameters.current_state.label // "unavailable"' 2>/dev/null || echo unavailable
}

# Reports a FAILED fault through the fault manager's report_fault service.
# Usage: report_fault CODE SOURCE_ID
report_fault() {
    curl -s -m 20 -X POST "${API_BASE}/apps/medkit-fault-manager/operations/report_fault/executions" \
        -H "Content-Type: application/json" \
        -d "$(jq -nc --arg code "$1" --arg src "$2" '{parameters: {fault_code: $code, event_type: 0,
            severity: 2, description: "smoke test", source_id: $src}}')" \
        | jq -e '.parameters.accepted == true' > /dev/null 2>&1
}

# Prints the fault code of every complete event watch-triggers.sh has logged.
# Usage: event_fault_codes LOG
event_fault_codes() {
    awk '/Event received:$/ { on = 1; buf = ""; next }
         on && /^---$/ { printf "%s", buf; on = 0; next }
         on { buf = buf $0 "\n" }' "$1" \
        | jq -rs '.[].payload.fault_code // empty' 2>/dev/null || true
}

# Proves watch-triggers.sh live: reports a fault from the anomaly detector's
# own node path, which fires the trigger, every 2 s until the watcher logs
# its event (max 20 s). Adds each reported code to PROBE_CODES.
# Usage: watcher_live PREFIX
PROBE_CODES=""
watcher_live() {
    local n=0 code
    while [ "$n" -lt 10 ]; do
        n=$((n + 1))
        code="${1}_${n}"
        PROBE_CODES="${PROBE_CODES} ${code}"
        report_fault "$code" "/bridge/anomaly_detector" || true
        sleep 2
        if grep -q "^${1}_" <<< "$(event_fault_codes "$WATCH_LOG")"; then
            return 0
        fi
    done
    return 1
}

# The inject needs a goal that Nav2 accepts and then aborts. An inactive
# navigator rejects the goal and no fault follows. Nav2 activates well after
# the gateway answers, so wait for the navigator and the planner.
echo "  Waiting for bt-navigator and planner-server to be active (max 180s)..."
nav2_state=""
elapsed=0
while [ "$elapsed" -lt 180 ]; do
    nav2_state="$(lifecycle_label bt-navigator)/$(lifecycle_label planner-server)"
    [ "$nav2_state" = "active/active" ] && break
    sleep 5
    elapsed=$((elapsed + 5))
done
if [ "$nav2_state" = "active/active" ]; then
    pass "bt-navigator and planner-server are active before the inject"
else
    fail "bt-navigator and planner-server are active before the inject" "states: ${nav2_state}"
fi

# The README says the trigger setup-triggers.sh creates does not fire for
# navigation goal faults. Watch it across the inject below as a user would.
TRIGGER_SETUP_OUTPUT=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash ./setup-triggers.sh 2>&1) || true
TRIGGER_ID=$(awk '/^  ID:/ { sub(/^  ID:[ \t]*/, ""); print; exit }' <<< "$TRIGGER_SETUP_OUTPUT")
WATCH_LOG=$(mktemp)
WATCH_PID=""
WATCH_LIVE_BEFORE=false
if [ -n "$TRIGGER_ID" ]; then
    pass "setup: setup-triggers.sh creates a trigger on apps/anomaly-detector"
    # exec makes the PID timeout's own; timeout passes a kill on to the whole stream.
    (cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" exec timeout 180 bash ./watch-triggers.sh "$TRIGGER_ID") \
        > "$WATCH_LOG" 2>&1 &
    WATCH_PID=$!
    if watcher_live "SMOKE_TRIGGER_BEFORE"; then
        WATCH_LIVE_BEFORE=true
        pass "setup: watch-triggers.sh is live before the navigation inject"
    else
        fail "setup: watch-triggers.sh is live before the navigation inject" "no probe event within 20 s"
    fi
else
    fail "setup: setup-triggers.sh creates a trigger on apps/anomaly-detector" \
        "$(tail -5 <<< "$TRIGGER_SETUP_OUTPUT")"
fi

# Inject a real fault via the Scripts API so check-entities.sh (section 6)
# and check-faults.sh exercise the fault-carrying fields, not just the
# empty case.
echo "  Injecting navigation failure via Scripts API..."
INJECT_RESPONSE=$(curl -s -m 30 -X POST "${API_BASE}/components/nav2-stack/scripts/inject-nav-failure/executions" \
    -H "Content-Type: application/json" -d '{"execution_type": "now"}') || true
INJECT_EXEC_ID=$(echo "$INJECT_RESPONSE" | jq -r '.id // empty')
if [ -n "$INJECT_EXEC_ID" ]; then
    elapsed=0
    while [ $elapsed -lt 30 ]; do
        st=$(curl -s "${API_BASE}/components/nav2-stack/scripts/inject-nav-failure/executions/${INJECT_EXEC_ID}" | jq -r '.status')
        if [ "$st" = "completed" ] || [ "$st" = "failed" ]; then
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
fi

echo "  Waiting for NAVIGATION_GOAL_ABORTED fault to appear (max 15s)..."
if poll_until "/faults" '.items[] | select(.fault_code == "NAVIGATION_GOAL_ABORTED")' 15; then
    pass "NAVIGATION_GOAL_ABORTED fault appeared in /faults"
else
    fail "NAVIGATION_GOAL_ABORTED fault appeared in /faults" "fault not found after 15s"
fi

# Only a watcher proven live before and after the navigation goal fault can
# show that the fault fired no event.
if [ -n "$WATCH_PID" ]; then
    WATCH_LIVE_AFTER=false
    if watcher_live "SMOKE_TRIGGER_AFTER"; then
        WATCH_LIVE_AFTER=true
        pass "setup: watch-triggers.sh is live after the navigation inject"
    else
        fail "setup: watch-triggers.sh is live after the navigation inject" "no probe event within 20 s"
    fi
    kill "$WATCH_PID" 2>/dev/null || true
    wait "$WATCH_PID" 2>/dev/null || true
    EVENT_CODES=$(event_fault_codes "$WATCH_LOG" | sort -u | paste -sd, -)
    if [ "$WATCH_LIVE_BEFORE" = true ] && [ "$WATCH_LIVE_AFTER" = true ]; then
        if grep -qE '^NAVIGATION_GOAL_' <<< "$(event_fault_codes "$WATCH_LOG")"; then
            fail "watch-triggers.sh gets no event for a navigation goal fault, as the README says" \
                "events carried: ${EVENT_CODES}"
        else
            pass "watch-triggers.sh gets no event for a navigation goal fault, as the README says"
        fi
    else
        fail "watch-triggers.sh gets no event for a navigation goal fault, as the README says" \
            "not checked: the watcher was not live before and after the inject; events carried: ${EVENT_CODES:-none}"
    fi
    curl -s -m 20 -o /dev/null -X DELETE "${API_BASE}/apps/anomaly-detector/triggers/${TRIGGER_ID}" || true
    for code in $PROBE_CODES; do
        curl -s -m 20 -o /dev/null -X DELETE "${API_BASE}/apps/anomaly-detector/faults/${code}" || true
    done
fi
rm -f "$WATCH_LOG"

# Prints the JSON values printed in section N of a script output as one array:
# [] when the section is missing, nothing when it is not JSON. Text after the
# section's last closing bracket, such as a script's closing lines, is dropped.
# Usage: script_section OUTPUT N
script_section() {
    awk -v hdr="^=== $2\\\\. " '
        $0 ~ hdr { on = 1; next }
        on && /^=== [0-9]+\. / { exit }
        on { lines[++n] = $0; if ($0 ~ /^[]}]/) last = n }
        END { for (i = 1; i <= last; i++) print lines[i] }' <<< "$1" \
        | sed -n '/^[[{]/,$p' | jq -s '.' 2>/dev/null || true
}

# Prints the fault list as the scripts show it, normalised by FILTER.
# Usage: api_faults FILTER
api_faults() {
    curl -s -m 20 "${API_BASE}/faults" | jq -c "[.items[] | $1] | sort_by(.code)" 2>/dev/null || true
}

# Passes when PRINTED is non-empty and equals EXPECTED or ALTERNATIVE.
# Usage: assert_printed_matches DESCRIPTION PRINTED EXPECTED [ALTERNATIVE]
assert_printed_matches() {
    local description="$1" printed="$2" expected="$3" alternative="${4:-}"
    if [ -n "$printed" ] && { [ "$printed" = "$expected" ] || [ "$printed" = "$alternative" ]; }; then
        pass "$description"
    else
        fail "$description" "printed ${printed:-nothing}; API ${expected:-unreadable}"
    fi
}

# The fault list can change while a script runs, so it is read before and
# after; the printed faults must equal one of the two reads.
ENTITY_FAULT_FIELDS='{code: .fault_code, severity: .severity_label, sources: .reporting_sources}'
FAULT_FIELDS='{code: .fault_code, severity: .severity_label, status: .status,
    sources: .reporting_sources, occurrences: .occurrence_count}'

# check-entities.sh without LiDAR data. The gateway keeps the last message of
# a topic it has read, and nothing above reads /scan, so with the simulator
# stopped the scan read has no message.
if stop_simulator; then
    # An error body also lacks .data, so only a successful read counts.
    if api_get "/apps/turtlebot3-node/data/scan" \
        && jq -e '.data | type == "object" and length == 0' <<< "$RESPONSE" > /dev/null 2>&1; then
        pass "setup: the /scan read succeeds and has no message while the simulator is stopped"
    else
        fail "setup: the /scan read succeeds and has no message while the simulator is stopped" \
            "$(head -c 200 <<< "$RESPONSE")"
    fi
    NO_SCAN_PLAIN=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" timeout 120 bash ./check-entities.sh < /dev/null 2>&1 \
        | sed 's/\x1b\[[0-9;]*m//g') || true
    resume_simulator
    if grep -q ': null' <<< "$NO_SCAN_PLAIN"; then
        fail "check-entities.sh prints no null fields without LiDAR data" \
            "$(grep -B1 ': null' <<< "$NO_SCAN_PLAIN" | head -10)"
    else
        pass "check-entities.sh prints no null fields without LiDAR data"
    fi
    if [ "$(script_section "$NO_SCAN_PLAIN" 5 | jq -c '.' 2>/dev/null)" = "[]" ] \
        && grep -q "LiDAR data not available" <<< "$NO_SCAN_PLAIN"; then
        pass "check-entities.sh prints the LiDAR hint without LiDAR data"
    else
        fail "check-entities.sh prints the LiDAR hint without LiDAR data" \
            "section 5: $(awk '/^=== 5\. /{on=1;next} /^=== 6\. /{exit} on' <<< "$NO_SCAN_PLAIN" | tr '\n' ';' | head -c 300)"
    fi
    if poll_until "/apps/turtlebot3-node/data/scan" '.data | length > 0' 30; then
        pass "setup: /scan has data again after the simulator resumes"
    else
        fail "setup: /scan has data again after the simulator resumes" "no scan data after 30 s"
    fi
else
    fail "setup: simulator stopped" "no gz sim process in ${DEMO_CONTAINER}"
fi

ENTITY_FAULTS_BEFORE=$(api_faults "$ENTITY_FAULT_FIELDS")
CHECK_ENTITIES_PLAIN=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash ./check-entities.sh 2>&1 \
    | sed 's/\x1b\[[0-9;]*m//g') || true
ENTITY_FAULTS_AFTER=$(api_faults "$ENTITY_FAULT_FIELDS")

if grep -q ': null' <<< "$CHECK_ENTITIES_PLAIN"; then
    fail "check-entities.sh prints no null fields" "$(grep -B1 ': null' <<< "$CHECK_ENTITIES_PLAIN" | head -10)"
else
    pass "check-entities.sh prints no null fields"
fi

# The scan's angle and range limits are fixed by the LiDAR model, so the
# printed ones must equal a direct read. Ranges past range_max are inf, which
# JSON carries as null, so only the count of sample ranges is checked.
SCAN_DIRECT=$(curl -s -m 20 "${API_BASE}/apps/turtlebot3-node/data/scan" | jq -c '.data' 2>/dev/null) || true
if script_section "$CHECK_ENTITIES_PLAIN" 5 | jq -e --argjson g "${SCAN_DIRECT:-null}" 'length == 1 and (.[0] as $p
        | ([$p.angle_min, $p.angle_max, $p.range_min, $p.range_max] | all(type == "number"))
        and $p.angle_min == $g.angle_min and $p.angle_max == $g.angle_max
        and $p.range_min == $g.range_min and $p.range_max == $g.range_max
        and ($p.sample_ranges | type == "array" and length == 5))' > /dev/null 2>&1; then
    pass "check-entities.sh section 5 shows the scan's values"
else
    fail "check-entities.sh section 5 shows the scan's values" \
        "printed $(script_section "$CHECK_ENTITIES_PLAIN" 5 | jq -c '.' 2>/dev/null | head -c 200); direct ${SCAN_DIRECT:0:200}"
fi

if grep -q "NAVIGATION_GOAL_ABORTED" <<< "$CHECK_ENTITIES_PLAIN"; then
    pass "check-entities.sh faults section shows the active fault code"
else
    fail "check-entities.sh faults section shows the active fault code" "NAVIGATION_GOAL_ABORTED not in output"
fi

assert_printed_matches "check-entities.sh lists every component id" \
    "$(script_section "$CHECK_ENTITIES_PLAIN" 2 | jq -c '[.[].id] | sort' 2>/dev/null)" \
    "$(curl -s -m 20 "${API_BASE}/components" | jq -c '[.items[].id] | sort' 2>/dev/null)"

assert_printed_matches "check-entities.sh shows each app with its component" \
    "$(script_section "$CHECK_ENTITIES_PLAIN" 3 | jq -c 'map({id, component}) | sort_by(.id)' 2>/dev/null)" \
    "$(curl -s -m 20 "${API_BASE}/apps" \
        | jq -c '[.items[] | {id, component: .["x-medkit"].component_id}] | sort_by(.id)' 2>/dev/null)"

assert_printed_matches "check-entities.sh shows every active fault with its severity and sources" \
    "$(script_section "$CHECK_ENTITIES_PLAIN" 6 | jq -c 'map({code, severity, sources}) | sort_by(.code)' 2>/dev/null)" \
    "$ENTITY_FAULTS_BEFORE" "$ENTITY_FAULTS_AFTER"

FAULTS_BEFORE=$(api_faults "$FAULT_FIELDS")
CHECK_FAULTS_PLAIN=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash ./check-faults.sh 2>&1 \
    | sed 's/\x1b\[[0-9;]*m//g') || true
FAULTS_AFTER=$(api_faults "$FAULT_FIELDS")

if grep -q ': null' <<< "$CHECK_FAULTS_PLAIN"; then
    fail "check-faults.sh prints no null fields" "$(grep -B1 ': null' <<< "$CHECK_FAULTS_PLAIN" | head -10)"
else
    pass "check-faults.sh prints no null fields"
fi

if grep -q "NAVIGATION_GOAL_ABORTED" <<< "$CHECK_FAULTS_PLAIN"; then
    pass "check-faults.sh shows the active fault code"
else
    fail "check-faults.sh shows the active fault code" "NAVIGATION_GOAL_ABORTED not in output"
fi

# check-faults.sh prints the faults between its "Active Faults:" and
# "Fault Summary:" lines.
assert_printed_matches "check-faults.sh shows every active fault with its severity, status, sources and count" \
    "$(awk '/Active Faults:$/ { on = 1; next } /Fault Summary:$/ { exit } on' <<< "$CHECK_FAULTS_PLAIN" \
        | sed -n '/^[[{]/,$p' | jq -cs 'map({code, severity, status, sources, occurrences}) | sort_by(.code)' 2>/dev/null)" \
    "$FAULTS_BEFORE" "$FAULTS_AFTER"

# Cleanup: clear all faults so smoke_test_navigation.sh (run next on this
# stack) does not inherit a latched fault confirmation.
echo "  Cleaning up: clearing faults..."
curl -s -X DELETE "${API_BASE}/faults" > /dev/null || true

section "Triggers"

assert_triggers_crud "apps" "diagnostic-bridge" "/api/v1/apps/diagnostic-bridge/faults"

section "check-faults.sh: a failed fault read is not reported as no faults"

FAILED_READ_LOG=$(mktemp)
if stop_fault_manager; then
    if api_get "/faults" 503; then
        pass "setup: GET /faults answers 503 while the fault manager is stopped"
    else
        fail "setup: GET /faults answers 503 while the fault manager is stopped" "got: $(head -c 200 <<< "${RESPONSE}")"
    fi
    if (cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" timeout 120 bash ./check-faults.sh) < /dev/null \
            > "$FAILED_READ_LOG" 2>&1; then
        FAILED_READ_RC=0
    else
        FAILED_READ_RC=$?
    fi
    resume_fault_manager
    if [ "$FAILED_READ_RC" -ne 0 ] && grep -q 'Could not read faults .*(HTTP 503)' "$FAILED_READ_LOG" \
        && ! grep -qE 'No active faults|Total active faults' "$FAILED_READ_LOG"; then
        pass "check-faults.sh exits non-zero on HTTP 503 and does not claim a fault count"
    else
        fail "check-faults.sh exits non-zero on HTTP 503 and does not claim a fault count" \
            "rc=${FAILED_READ_RC}; output: $(grep -vE '^ *$' "$FAILED_READ_LOG" | tail -n 6 | tr '\n' ';')"
    fi
    if poll_until "/faults" '.items | arrays' 30; then
        pass "setup: GET /faults answers again after the fault manager resumes"
    else
        fail "setup: GET /faults answers again after the fault manager resumes" "still failing after 30 s"
    fi
else
    fail "setup: fault manager stopped" "no fault_manager_node process in ${DEMO_CONTAINER}"
fi
rm -f "$FAILED_READ_LOG"

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
