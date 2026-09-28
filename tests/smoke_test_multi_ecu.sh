#!/bin/bash
# Smoke tests for multi_ecu_aggregation demo
# Runs from the host against the perception ECU gateway on localhost:8080
# The perception ECU is the primary aggregator - it should expose local entities
# plus aggregated entities from planning and actuation ECUs.
# Reads the demo's container scripts and config from this checkout and runs its
# host-side wrapper scripts.
#
# Usage: ./tests/smoke_test_multi_ecu.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
# shellcheck disable=SC2034  # Used by smoke_lib.sh
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

# --- Wait for gateway startup ---

# Multi-ECU demo needs extra time: 3 containers building their ROS graphs
# plus aggregation discovery across the Docker network
wait_for_gateway 120

# Wait for runtime node linking (perception ECU local nodes)
wait_for_runtime_linking "/apps/lidar-driver/data" 90

# Wait for aggregation to discover peer ECUs.
# The aggregator pulls each peer independently, so a single peer's marker app is
# not a sufficient readiness gate: we must see a representative app from EACH
# peer ECU before the discovery assertions run. Otherwise a slower peer
# (typically the actuation ECU) races the checks and surfaces as missing apps
# ("found 7" instead of >=10).
#   planning ECU  -> path-planner
#   actuation ECU -> motor-controller
echo "  Waiting for aggregated entities from planning + actuation ECUs (max 60s each)..."
for peer in "planning ECU:path-planner" "actuation ECU:motor-controller"; do
    peer_name="${peer%%:*}"
    peer_app="${peer##*:}"
    if poll_until "/apps" ".items[] | select(.id == \"${peer_app}\")" 60; then
        echo -e "  ${GREEN}${peer_name} aggregated (${peer_app})${NC}"
    else
        echo -e "  ${RED}${peer_name} not aggregated within 60s (${peer_app} missing)${NC}"
        exit 1
    fi
done

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

section "Entity Discovery - Components"

# robot-alpha is the top-level parent component shared across all 3 ECUs
if api_get "/components"; then
    if echo "$RESPONSE" | items_contain_id "robot-alpha"; then
        pass "components contains 'robot-alpha'"
    else
        fail "components contains 'robot-alpha'" "not found in response"
    fi
else
    fail "GET /components returns 200" "unexpected status code"
fi

# ECU components are sub-components of robot-alpha (parent_component_id set)
# They appear under /components/robot-alpha/subcomponents, not /components
if api_get "/components/robot-alpha/subcomponents"; then
    for comp_id in perception-ecu planning-ecu actuation-ecu; do
        if echo "$RESPONSE" | items_contain_id "$comp_id"; then
            pass "subcomponents contains '${comp_id}'"
        else
            fail "subcomponents contains '${comp_id}'" "not found in response"
        fi
    done
else
    fail "GET /components/robot-alpha/subcomponents returns 200" "unexpected status code"
fi

section "Entity Discovery - Apps"

# 10 demo apps across 3 ECUs:
#   Perception: lidar-driver, camera-driver, point-cloud-filter, object-detector
#   Planning:   path-planner, behavior-planner, task-scheduler
#   Actuation:  motor-controller, joint-driver, gripper-controller
if api_get "/apps"; then
    for app_id in \
        lidar-driver camera-driver point-cloud-filter object-detector \
        path-planner behavior-planner task-scheduler \
        motor-controller joint-driver gripper-controller; do
        if echo "$RESPONSE" | items_contain_id "$app_id"; then
            pass "apps contains '${app_id}'"
        else
            fail "apps contains '${app_id}'" "not found in response"
        fi
    done
    # Verify at least 10 demo apps are present
    local_count=$(echo "$RESPONSE" | jq '[.items[] | select(.id | test("^(lidar|camera|point|object|path|behavior|task|motor|joint|gripper)"))] | length')
    if [ "$local_count" -ge 10 ]; then
        pass "at least 10 demo apps discovered"
    else
        fail "at least 10 demo apps discovered" "found ${local_count}"
    fi
else
    fail "GET /apps returns 200" "unexpected status code"
fi

section "Entity Discovery - Functions"

# 3 cross-ECU functions: autonomous-navigation, obstacle-avoidance, task-execution
if api_get "/functions"; then
    for func_id in autonomous-navigation obstacle-avoidance task-execution; do
        if echo "$RESPONSE" | items_contain_id "$func_id"; then
            pass "functions contains '${func_id}'"
        else
            fail "functions contains '${func_id}'" "not found in response"
        fi
    done
else
    fail "GET /functions returns 200" "unexpected status code"
fi

section "Data Access"

# Test data access on a local perception app
assert_non_empty_items "/apps/lidar-driver/data"

section "Configurations"

# Test configurations on a local perception app
assert_non_empty_items "/apps/lidar-driver/configurations"

section "Faults"

if api_get "/faults"; then
    pass "GET /faults returns 200"
else
    fail "GET /faults returns 200" "unexpected status code"
fi

section "Scripts"

# Perception ECU scripts should include inject-sensor-failure
assert_scripts_list "perception-ecu" "inject-sensor-failure"

section "Container Script Parameter Writes"

# The checks below read what each container script writes from the scripts
# themselves and the launch values from the demo config and node sources.
DEMO_DIR="$(cd "${SCRIPT_DIR}/../demos/multi_ecu_aggregation" && pwd)"
CONTAINER_SCRIPTS="${DEMO_DIR}/container_scripts"

# Container scripts this test executes. Must match container_scripts/.
EXERCISED_SCRIPTS="actuation-ecu/inject-gripper-jam actuation-ecu/restore-normal \
perception-ecu/inject-sensor-failure perception-ecu/restore-normal \
planning-ecu/inject-planning-delay planning-ecu/restore-normal"

# Parameter writes of a container script, one "app param value" line each,
# read from its top-level put_config calls.
script_writes() {
    sed -n -E 's/^put_config ([a-z-]+) ([a-z_]+) ([^ ]+)$/\1 \2 \3/p' \
        "${CONTAINER_SCRIPTS}/$1/$2/script.bash"
}

# Value a parameter has after launch: set in the ECU params file, else the
# node's declared default. An app id is its node name with dashes.
launch_value() {
    local ecu="$1" app="$2" param="$3" node value
    node="${app//-/_}"
    value=$(awk -v node="${node}:" -v key="${param}:" '
        /^[^ #]/ { in_node = 0 }
        /^  [^ #]/ { in_node = ($1 == node) }
        in_node && $1 == key { print $2; exit }
    ' "${DEMO_DIR}/config/${ecu%-ecu}_params.yaml" 2>/dev/null) || value=""
    if [ -z "$value" ]; then
        value=$(sed -n -E "s/.*declare_parameter\(\"${param}\", ([^)]+)\).*/\1/p" \
            "${DEMO_DIR}"/src/*/"${node}.cpp" 2>/dev/null | head -1) || value=""
    fi
    echo "$value"
}

# Every parameter a script writes holds the expected value, read back with a
# plain GET. $3 is "written" (the value the script writes) or "launch".
assert_script_params() {
    local ecu="$1" script="$2" expect="$3" writes app param value want
    writes=$(script_writes "$ecu" "$script") || writes=""
    if [ -z "$writes" ]; then
        fail "${ecu}/${script} writes parameters the test can read" "no put_config calls found"
        return
    fi
    while read -r app param value; do
        want="$value"
        if [ "$expect" = "launch" ]; then
            want=$(launch_value "$ecu" "$app" "$param")
        fi
        local label="after ${script} on ${ecu}: ${app}/${param} is ${want:-<unknown>}"
        if [ -z "$want" ]; then
            fail "$label" "no launch value found in the params file or the node source"
        elif api_get "/apps/${app}/configurations/${param}" \
            && echo "$RESPONSE" | jq -e --argjson want "$want" '.data == $want' > /dev/null 2>&1; then
            pass "$label"
        else
            fail "$label" "got: $(echo "$RESPONSE" | jq -c 'if has("data") then .data else . end' 2>/dev/null \
                || echo "$RESPONSE")"
        fi
    done <<< "$writes"
}

listed_scripts=$(cd "$CONTAINER_SCRIPTS" && find . -name script.bash | sed -e 's:^\./::' -e 's:/script\.bash$::' \
    | sort | tr '\n' ' ')
exercised_scripts=$(echo "$EXERCISED_SCRIPTS" | tr ' ' '\n' | sed '/^$/d' | sort | tr '\n' ' ')
if [ "$listed_scripts" = "$exercised_scripts" ]; then
    pass "this test executes every container script"
else
    fail "this test executes every container script" "container_scripts: ${listed_scripts}"
fi

# script_writes reads only top-level put_config calls. A parameter write in any
# other form fails here instead of going unchecked.
for script_file in "${CONTAINER_SCRIPTS}"/*/*/script.bash; do
    script_name="${script_file#"${CONTAINER_SCRIPTS}"/}"
    script_name="${script_name%/script.bash}"
    read_calls=$(grep -cE '^put_config [a-z-]+ [a-z_]+ [^ ]+$' "$script_file" || true)
    all_calls=$(grep -cE '(^|[^[:alnum:]_])put_config[[:space:]]' "$script_file" || true)
    config_urls=$(grep -c '/configurations/' "$script_file" || true)
    if [ "$read_calls" -gt 0 ] && [ "$read_calls" -eq "$all_calls" ] && [ "$config_urls" -eq 1 ]; then
        pass "${script_name} writes parameters only through put_config calls"
    else
        fail "${script_name} writes parameters only through put_config calls" \
            "top-level put_config: ${read_calls}, all put_config calls: ${all_calls}, configuration URLs: ${config_urls}"
    fi
done

# An inject must change what it writes, and the ECU's restore-normal must
# write it back.
for inject_dir in "${CONTAINER_SCRIPTS}"/*/inject-*/; do
    ecu=$(basename "$(dirname "$inject_dir")")
    inject=$(basename "$inject_dir")
    restore_writes=$(script_writes "$ecu" "restore-normal") || restore_writes=""
    while read -r app param value; do
        [ -n "$app" ] || continue
        launch=$(launch_value "$ecu" "$app" "$param")
        if [ -n "$launch" ] && jq -n -e --argjson a "$value" --argjson b "$launch" '$a != $b' > /dev/null 2>&1; then
            pass "${ecu}/${inject} changes ${app}/${param} from its launch value ${launch}"
        else
            fail "${ecu}/${inject} changes ${app}/${param} from its launch value" \
                "writes ${value}, launch value ${launch:-<unknown>}"
        fi
        if echo "$restore_writes" | grep -qE "^${app} ${param} "; then
            pass "${ecu}/restore-normal writes ${app}/${param} back"
        else
            fail "${ecu}/restore-normal writes ${app}/${param} back" "not among its put_config calls"
        fi
    done <<< "$(script_writes "$ecu" "$inject")"
done

section "Script Injection and Restore"

# The first script executions on every ECU happen here, with no earlier script
# or ros2 CLI call in that container.

# Runs a container script through the Scripts API and prints its execution
# once it has ended. Returns 1 if it does not end within $3 seconds.
run_script() {
    local endpoint="/components/$1/scripts/$2/executions" deadline exec_id
    exec_id=$(curl -s -m 30 -X POST "${API_BASE}${endpoint}" -H "Content-Type: application/json" \
        -d '{"execution_type": "now"}' | jq -r '.id // empty') || exec_id=""
    [ -n "$exec_id" ] || return 1
    deadline=$((SECONDS + $3))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if api_get "${endpoint}/${exec_id}" \
            && echo "$RESPONSE" | jq -e '.status | IN("completed", "failed", "terminated")' > /dev/null 2>&1; then
            echo "$RESPONSE"
            return 0
        fi
        sleep 1
    done
    return 1
}

# Latest path path-planner published. Its header stamp is taken at the end of
# the planning cycle, after the injected delay.
PATH_DATA="/apps/path-planner/data/%2Fplanning%2Fpath"

# Seconds between two consecutive paths from path-planner, waiting at most $1 s.
path_period() {
    local deadline=$((SECONDS + $1)) first="" stamp
    while [ "$SECONDS" -lt "$deadline" ]; do
        if api_get "$PATH_DATA"; then
            stamp=$(echo "$RESPONSE" | jq -r '.data.header.stamp | .sec + .nanosec / 1e9' 2>/dev/null) || stamp=""
            if [ -z "$first" ]; then
                first="$stamp"
            elif [ -n "$stamp" ] && [ "$stamp" != "$first" ]; then
                jq -n --argjson a "$first" --argjson b "$stamp" '$b - $a'
                return 0
            fi
        fi
        sleep 0.2
    done
    return 1
}

# Perception ECU
assert_script_execution "perception-ecu" "inject-sensor-failure"
assert_script_params "perception-ecu" "inject-sensor-failure" written
assert_script_execution "perception-ecu" "restore-normal"
assert_script_params "perception-ecu" "restore-normal" launch

# Planning ECU
DELAY_MS=$(script_writes "planning-ecu" "inject-planning-delay" \
    | awk '$1 == "path-planner" && $2 == "planning_delay_ms" { print $3 }') || DELAY_MS=""
DELAY_MS="${DELAY_MS:-0}"

assert_script_execution "planning-ecu" "inject-planning-delay"
assert_script_params "planning-ecu" "inject-planning-delay" written

if poll_until "/faults" '.items[] | select(.fault_code == "PATH_PLANNER")' 30; then
    pass "PATH_PLANNER fault is reported after inject-planning-delay"
else
    fail "PATH_PLANNER fault is reported after inject-planning-delay" "not reported within 30s"
fi

if period=$(path_period 20) \
    && jq -n -e --argjson p "$period" --argjson d "$DELAY_MS" '$d > 0 and $p >= $d / 1000 * 0.8' > /dev/null; then
    pass "path-planner publishes one path per ${DELAY_MS} ms or slower while the delay is injected (${period}s)"
else
    fail "path-planner publishes one path per ${DELAY_MS} ms or slower while the delay is injected" \
        "period: ${period:-no second path within 20s}"
fi

# The planning cycle is still waiting out the delay here, and restore-normal
# must not wait with it.
RESTORE_BOUND_SEC=15
restore_start=$SECONDS
exec_json=$(run_script "planning-ecu" "restore-normal" "$RESTORE_BOUND_SEC") || exec_json=""
restore_status=$(echo "$exec_json" | jq -r '.status // empty' 2>/dev/null) || restore_status=""
if [ "$restore_status" = "completed" ]; then
    pass "planning restore-normal after inject-planning-delay completes within ${RESTORE_BOUND_SEC}s \
($((SECONDS - restore_start))s)"
else
    fail "planning restore-normal after inject-planning-delay completes within ${RESTORE_BOUND_SEC}s" \
        "status after $((SECONDS - restore_start))s: ${restore_status:-still running} $(echo "$exec_json" \
        | jq -c '.error // empty' 2>/dev/null)"
fi
assert_script_params "planning-ecu" "restore-normal" launch

if period=$(path_period 10) \
    && jq -n -e --argjson p "$period" --argjson d "$DELAY_MS" '$p < $d / 1000 / 2' > /dev/null; then
    pass "path-planner is back to its planning rate after restore-normal (${period}s)"
else
    fail "path-planner is back to its planning rate after restore-normal" "period: ${period:-no second path within 10s}"
fi

# Actuation ECU
assert_script_execution "actuation-ecu" "inject-gripper-jam"
assert_script_params "actuation-ecu" "inject-gripper-jam" written
assert_script_execution "actuation-ecu" "restore-normal"
assert_script_params "actuation-ecu" "restore-normal" launch

# The perception ECU aggregates the fault lists of all three ECUs.
if poll_until "/faults" '.items | length == 0' 15; then
    pass "no faults remain after restore-normal on all three ECUs"
else
    fail "no faults remain after restore-normal on all three ECUs" "found: $(echo "$RESPONSE" | jq -c '.items')"
fi

section "Host Inject and Restore Wrappers"

if host_out=$(GATEWAY_URL="$GATEWAY_URL" "${DEMO_DIR}/inject-cascade-failure.sh" 2>&1); then
    pass "inject-cascade-failure.sh exits 0"
else
    fail "inject-cascade-failure.sh exits 0" "$(echo "$host_out" | tail -5)"
fi
assert_script_params "perception-ecu" "inject-sensor-failure" written
assert_script_params "planning-ecu" "inject-planning-delay" written
assert_script_params "actuation-ecu" "inject-gripper-jam" written

if poll_until "/faults" '.items[] | select(.fault_code == "PATH_PLANNER")' 30; then
    pass "PATH_PLANNER fault is reported after inject-cascade-failure.sh"
else
    fail "PATH_PLANNER fault is reported after inject-cascade-failure.sh" "not reported within 30s"
fi

# restore-normal.sh waits up to 120s for each ECU; with the delay injected it
# must still finish far inside that.
HOST_RESTORE_BOUND_SEC=30
host_start=$SECONDS
if host_out=$(GATEWAY_URL="$GATEWAY_URL" "${DEMO_DIR}/restore-normal.sh" 2>&1); then
    host_rc=0
else
    host_rc=$?
fi
host_took=$((SECONDS - host_start))
if [ "$host_rc" -eq 0 ] && [ "$host_took" -le "$HOST_RESTORE_BOUND_SEC" ]; then
    pass "restore-normal.sh exits 0 within ${HOST_RESTORE_BOUND_SEC}s with a planning delay injected (${host_took}s)"
else
    fail "restore-normal.sh exits 0 within ${HOST_RESTORE_BOUND_SEC}s with a planning delay injected" \
        "exit ${host_rc} after ${host_took}s: $(echo "$host_out" | tail -5)"
fi
assert_script_params "perception-ecu" "restore-normal" launch
assert_script_params "planning-ecu" "restore-normal" launch
assert_script_params "actuation-ecu" "restore-normal" launch

if poll_until "/faults" '.items | length == 0' 15; then
    pass "no faults remain after restore-normal.sh"
else
    fail "no faults remain after restore-normal.sh" "found: $(echo "$RESPONSE" | jq -c '.items')"
fi

section "Parameter Write Failure Reporting"

# A lock on gripper-controller's configurations held by another client makes
# the gateway refuse those writes from the script, while the other actuation
# writes still land.
LOCK_CLIENT="smoke-test-write-failure"
lock_id=$(curl -s -m 30 -X POST "${API_BASE}/apps/gripper-controller/locks" \
    -H "X-Client-Id: ${LOCK_CLIENT}" -H "Content-Type: application/json" \
    -d '{"lock_expiration": 60, "scopes": ["configurations"]}' | jq -r '.id // empty') || lock_id=""
if [ -n "$lock_id" ]; then
    pass "another client locks gripper-controller configurations"
else
    fail "another client locks gripper-controller configurations" "no lock id returned"
fi

if exec_json=$(run_script "actuation-ecu" "restore-normal" 60); then
    exec_status=$(echo "$exec_json" | jq -r '.status')
    exec_message=$(echo "$exec_json" | jq -r '.error.message // ""')
    if [ "$exec_status" = "failed" ]; then
        pass "actuation restore-normal fails while gripper-controller writes are refused"
    else
        fail "actuation restore-normal fails while gripper-controller writes are refused" "status: ${exec_status}"
    fi
    while read -r app param value; do
        if [ "$app" = "gripper-controller" ]; then
            if echo "$exec_message" | grep -qF "FAIL: ${app}/${param}"; then
                pass "failure message names the refused write ${app}/${param}"
            else
                fail "failure message names the refused write ${app}/${param}" "message: ${exec_message}"
            fi
        elif echo "$exec_message" | grep -qF "FAIL: ${app}/${param}"; then
            fail "failure message does not name ${app}/${param}, which succeeded" "message: ${exec_message}"
        else
            pass "failure message does not name ${app}/${param}, which succeeded"
        fi
    done <<< "$(script_writes "actuation-ecu" "restore-normal")"
else
    fail "actuation restore-normal ends while gripper-controller writes are refused" "no end state within 60s"
fi

if [ -n "$lock_id" ]; then
    unlock_status=$(curl -s -m 30 -o /dev/null -w "%{http_code}" -X DELETE \
        "${API_BASE}/apps/gripper-controller/locks/${lock_id}" -H "X-Client-Id: ${LOCK_CLIENT}") || true
    if [ "$unlock_status" = "204" ]; then
        pass "gripper-controller lock is released"
    else
        fail "gripper-controller lock is released" "got HTTP ${unlock_status}"
    fi
fi

# Leave the demo restored.
assert_script_execution "actuation-ecu" "restore-normal"
assert_script_params "actuation-ecu" "restore-normal" launch

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
