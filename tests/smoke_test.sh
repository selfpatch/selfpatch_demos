#!/bin/bash
# Smoke tests for sensor_diagnostics demo
# Runs from the host against the containerized gateway on localhost:8080. Needs
# docker access to the demo container (DEMO_CONTAINER) to pause its fault
# manager and to restart it.
#
# Usage: ./tests/smoke_test.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

DEMO_CONTAINER="${DEMO_CONTAINER:-sensor_diagnostics_demo_ci}"
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

LIDAR_PATTERN='^/root/demo_ws/install/sensor_diagnostics_demo/lib/sensor_diagnostics_demo/lidar_sim_node '

# SIGSTOP on the LiDAR node keeps it in the ROS graph but stops its scans.
stop_lidar() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${LIDAR_PATTERN}') || exit 1
        kill -STOP \"\${pid}\"
    " > /dev/null 2>&1
}

resume_lidar() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${LIDAR_PATTERN}') || exit 0
        kill -CONT \"\${pid}\"
    " > /dev/null 2>&1 || true
}

# print_summary reads the script's exit status from $?, so hand it the status
# saved on entry. set +e: under errexit `(exit rc)` would end the trap before
# print_summary runs.
cleanup_on_exit() {
    local rc=$?
    set +e
    resume_fault_manager
    resume_lidar
    (exit "${rc}")
    print_summary
}
trap cleanup_on_exit EXIT

CHECK_DEMO_SCRIPT="${SCRIPT_DIR}/../demos/sensor_diagnostics/check-demo.sh"

# Runs check-demo.sh and prints its output without ANSI colours.
run_check_demo() {
    GATEWAY_URL="$GATEWAY_URL" bash "$CHECK_DEMO_SCRIPT" 2>&1 | sed 's/\x1b\[[0-9;]*m//g'
}

# Runs check-demo.sh with DATA_WAIT_SEC=$1 (empty: the script's default) and
# prints each output line without ANSI colours, prefixed with the
# milliseconds since the run started.
run_check_demo_timed() {
    local start=${EPOCHREALTIME//[!0-9]/} line
    env ${1:+"DATA_WAIT_SEC=$1"} GATEWAY_URL="$GATEWAY_URL" timeout 180 bash "$CHECK_DEMO_SCRIPT" < /dev/null 2>&1 \
        | while IFS= read -r line; do
            printf '%d %s\n' $(( (${EPOCHREALTIME//[!0-9]/} - start) / 1000 )) "$line"
        done | sed 's/\x1b\[[0-9;]*m//g'
}

# Prints the milliseconds check-demo.sh spent between the health check and
# section 1, from a run_check_demo_timed output. Without section 1, the time
# of the last line.
readiness_wait_ms() {
    awk '/^[0-9]+ .*Gateway is healthy/ { s = $1 }
         /^[0-9]+ === 1\. / { e = $1; exit }
         { last = $1 }
         END { if (e == "") e = last; print e - s }' <<< "$1"
}

# Drops the time prefix of a run_check_demo_timed output.
untimed() {
    awk '{ sub(/^[0-9]+ /, ""); print }' <<< "$1"
}

# Prints what check-demo.sh printed between the health check and section 1.
readiness_wait_output() {
    untimed "$1" | awk '/Gateway is healthy/ { on = 1; next } /^=== 1\. / { exit } on'
}

# Prints the text lines of section N of a check-demo.sh output.
check_demo_section_text() {
    awk -v hdr="^=== $2\\\\. " '
        $0 ~ hdr { on = 1; next }
        on && /^=== [0-9]+\. / { exit }
        on' <<< "$1"
}

# Prints the JSON values printed in section N of a check-demo.sh output as
# one array: [] when the section is missing, nothing when it is not JSON.
# Usage: check_demo_section OUTPUT N
check_demo_section() {
    check_demo_section_text "$1" "$2" | sed -n '/^[[{]/,$p' | jq -s '.' 2>/dev/null || true
}

# Prints the value of a configuration, or nothing when it cannot be read.
# Usage: config_value APP PARAM
config_value() {
    curl -s -m 10 "${API_BASE}/apps/$1/configurations/$2" | jq -c '.data' 2>/dev/null || true
}

# True when section N (5 LiDAR, 6 IMU, 7 GPS, 8 configurations) of a
# check-demo.sh output carries values of the right type.
# Usage: section_values_printed OUTPUT N
section_values_printed() {
    local filter
    case "$2" in
        5) filter='length == 1 and ([.[0] | .angle_min, .angle_max, .range_min, .range_max, .sample_ranges[]]
               | length == 9 and all(type == "number"))' ;;
        6) filter='length == 1 and ([.[0] | .linear_acceleration[], .angular_velocity[]]
               | length == 6 and all(type == "number"))' ;;
        7) filter='length == 1 and ([.[0] | .latitude, .longitude, .altitude] | all(type == "number"))' ;;
        8) filter='length > 0' ;;
    esac
    check_demo_section "$1" "$2" | jq -e "$filter" > /dev/null 2>&1
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

# --- Wait for gateway startup ---

# /health answers about a second before the gateway links the sensor nodes, and
# wait_for_gateway polls only every 2 s. The tighter poll lets the first
# check-demo.sh run below start inside that window on a demo that has just started.
for _ in $(seq 1 450); do
    curl -sf -m 2 "${API_BASE}/health" > /dev/null 2>&1 && break
    sleep 0.2
done
wait_for_gateway 90

section "Check-Demo Before Linking"

# Until the sensor nodes are linked, their data reads come back empty.
# check-demo.sh must wait for the links and then print real values.
EARLY_LIDAR_ITEMS=$(curl -s -m 5 "${API_BASE}/apps/lidar-sim/data" | jq '.items | length' 2>/dev/null) || true
EARLY_RC=0
EARLY_TIMED=$(run_check_demo_timed "") || EARLY_RC=$?
EARLY_PLAIN=$(untimed "$EARLY_TIMED")
echo "  LiDAR data items when check-demo.sh started: ${EARLY_LIDAR_ITEMS:-unreadable}; exit code ${EARLY_RC}"
echo "  readiness wait $(readiness_wait_ms "$EARLY_TIMED") ms:" \
    "$(readiness_wait_output "$EARLY_TIMED" | grep -vE '^ *$' | tr '\n' ';' | head -c 300)"

if grep -q ': null' <<< "$EARLY_PLAIN"; then
    fail "check-demo.sh started right after /health prints no null fields" \
         "$(grep -B1 ': null' <<< "$EARLY_PLAIN" | head -10)"
else
    pass "check-demo.sh started right after /health prints no null fields"
fi

EARLY_MISSING=""
for n in 5 6 7 8; do
    section_values_printed "$EARLY_PLAIN" "$n" || EARLY_MISSING="${EARLY_MISSING} ${n}"
done
if [ "$EARLY_RC" -eq 0 ] && [ -z "$EARLY_MISSING" ]; then
    pass "check-demo.sh started right after /health exits 0 and prints values in sections 5-8"
else
    fail "check-demo.sh started right after /health exits 0 and prints values in sections 5-8" \
         "exit code ${EARLY_RC}; sections without values:${EARLY_MISSING:- none}; last lines: $(grep -vE '^ *$' <<< "$EARLY_PLAIN" | tail -n 3 | tr '\n' ';')"
fi

# Wait for entity discovery + runtime linking (nodes need to be linked to manifest apps)
# In hybrid mode, manifest entities appear instantly but data/configurations require
# the runtime refresh cycle to link ROS 2 nodes to manifest apps.
wait_for_runtime_linking "/apps/lidar-sim/data" 60

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

test_entity_discovery "areas" sensors processing diagnostics bridge
test_entity_discovery "components" lidar-unit imu-unit gps-unit camera-unit compute-unit gateway fault-manager diagnostic-bridge-unit
test_entity_discovery "apps" lidar-sim imu-sim gps-sim camera-sim anomaly-detector medkit-gateway medkit-fault-manager diagnostic-bridge
test_entity_discovery "functions" sensor-monitoring anomaly-detection fault-management

section "Discovery Relationships"

assert_non_empty_items "/areas/sensors/components"

section "Linux Introspection"

assert_procfs_introspection "lidar-sim"

section "Data Access"

assert_non_empty_items "/apps/lidar-sim/data"

section "Configurations"

assert_non_empty_items "/apps/lidar-sim/configurations"

if echo "$RESPONSE" | jq -e '.items[] | select(.name == "noise_stddev")' > /dev/null 2>&1; then
    pass "configurations contains 'noise_stddev' parameter"
else
    fail "configurations contains 'noise_stddev' parameter" "not found in response"
fi

section "Operations"

# fault_manager services may take extra time to be discovered via runtime graph introspection
echo "  Waiting for fault-manager operations to appear (max 30s)..."
if poll_until "/apps/medkit-fault-manager/operations" '.items | length > 0' 30; then
    pass "GET /apps/medkit-fault-manager/operations returns non-empty items"
else
    fail "GET /apps/medkit-fault-manager/operations returns non-empty items" "items still empty after 30s"
fi

section "Scripts"

assert_scripts_list "compute-unit" "run-diagnostics"
assert_script_execution "compute-unit" "run-diagnostics" 30

section "Bulk Data"

# Bulk data endpoint should return 200 with categories list (may be empty without faults)
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

section "Check-Demo Sensor Values"

# Runs before the fault injection below, while the noise is at its default: at
# the injected 0.5 m, 8 sigma spans the whole LiDAR range.

# Checks section N of the first run against a direct read of ENDPOINT, and
# that the second run printed a different sample. FILTER sees the printed
# values as input, the direct read's .data as $g and SIGMA as $sigma.
# Usage: assert_section_live N DESCRIPTION ENDPOINT SIGMA FILTER
assert_section_live() {
    local n="$1" description="$2" endpoint="$3" sigma="$4" filter="$5"
    local printed printed_again direct
    printed=$(check_demo_section "$SENSOR_RUN_1" "$n")
    printed_again=$(check_demo_section "$SENSOR_RUN_2" "$n")
    if ! api_get "$endpoint"; then
        fail "$description" "direct read of ${endpoint} failed"
        return
    fi
    direct=$(jq -c '.data' <<< "$RESPONSE" 2>/dev/null) || true
    if ! jq -e --argjson g "${direct:-null}" --argjson sigma "${sigma:-null}" "$filter" \
            <<< "$printed" > /dev/null 2>&1; then
        fail "$description" "printed $(jq -c '.' <<< "$printed" 2>/dev/null || echo "no JSON"); sigma ${sigma}; direct read ${direct:0:300}"
    elif [ "$printed" = "$printed_again" ]; then
        fail "$description" "two runs printed the same values: $(jq -c '.' <<< "$printed")"
    else
        pass "$description"
    fi
}

SENSOR_RUN_1=$(run_check_demo) || true
# Every sensor publishes at 1 Hz or faster, so a run a second later reads new samples.
sleep 1
SENSOR_RUN_2=$(run_check_demo) || true

if grep -q ': null' <<< "$SENSOR_RUN_1"; then
    fail "check-demo.sh prints no null fields" "$(grep -B1 ': null' <<< "$SENSOR_RUN_1" | head -10)"
else
    pass "check-demo.sh prints no null fields"
fi

# Tolerances are 8 sigma of the live noise configuration: two independent
# samples differ by more than that with a probability below 1e-7. The LiDAR
# check also requires 8 sigma below a tenth of the scan's range span, or it
# could not tell a real range from an invented one.
LIDAR_FILTER=$(cat <<'JQ'
8 * $sigma < ($g.range_max - $g.range_min) / 10
and length == 1 and (.[0] as $p
  | $p.angle_min == $g.angle_min and $p.angle_max == $g.angle_max
    and $p.range_min == $g.range_min and $p.range_max == $g.range_max
    and ($p.sample_ranges | type == "array" and length == 5)
    and ([range(5)] | all(. as $i | $p.sample_ranges[$i] as $r
          | ($r | type) == "number"
            and $r >= $g.range_min and $r <= $g.range_max
            and (($r - $g.ranges[$i]) | fabs) <= 8 * $sigma)))
JQ
)
IMU_FILTER=$(cat <<'JQ'
length == 1 and (.[0] as $p
  | [["linear_acceleration", $sigma.accel], ["angular_velocity", $sigma.gyro]]
  | all(.[0] as $k | .[1] as $s | ["x", "y", "z"]
        | all(. as $a | ($p[$k][$a] | type) == "number"
              and (($p[$k][$a] - $g[$k][$a]) | fabs) <= 8 * $s)))
JQ
)
# 111 km per degree of latitude; a degree of longitude shrinks with cos(latitude).
GPS_FILTER=$(cat <<'JQ'
length == 1 and (.[0] as $p
  | ([$p.latitude, $p.longitude, $p.altitude] | all(type == "number"))
    and (($p.latitude - $g.latitude) | fabs) <= 8 * $sigma.pos / 111000
    and (($p.longitude - $g.longitude) | fabs)
        <= 8 * $sigma.pos / (111000 * (($g.latitude * 3.141592653589793 / 180) | cos))
    and (($p.altitude - $g.altitude) | fabs) <= 8 * $sigma.alt
    and $p.status == $g.status)
JQ
)

assert_section_live 5 "check-demo.sh section 5 shows live LiDAR values matching a direct read" \
    "/apps/lidar-sim/data/sensors%2Fscan" "$(config_value lidar-sim noise_stddev)" "$LIDAR_FILTER"
assert_section_live 6 "check-demo.sh section 6 shows live IMU values matching a direct read" \
    "/apps/imu-sim/data/sensors%2Fimu" \
    "{\"accel\": $(config_value imu-sim accel_noise_stddev), \"gyro\": $(config_value imu-sim gyro_noise_stddev)}" \
    "$IMU_FILTER"
assert_section_live 7 "check-demo.sh section 7 shows live GPS values matching a direct read" \
    "/apps/gps-sim/data/sensors%2Ffix" \
    "{\"pos\": $(config_value gps-sim position_noise_stddev), \"alt\": $(config_value gps-sim altitude_noise_stddev)}" \
    "$GPS_FILTER"

section "Fault Injection"

# Inject noise fault via configuration API
echo "  Injecting noise fault (noise_stddev=0.5)..."
INJECT_STATUS=$(curl -s -o /dev/null -w "%{http_code}" \
    -X PUT "${API_BASE}/apps/lidar-sim/configurations/noise_stddev" \
    -H "Content-Type: application/json" \
    -d '{"value": 0.5}') || true

if [ -z "$INJECT_STATUS" ]; then
    fail "PUT noise_stddev=0.5 returns 200" "request failed (no HTTP status received)"
elif [ "$INJECT_STATUS" = "200" ]; then
    pass "PUT noise_stddev=0.5 returns 200"
else
    fail "PUT noise_stddev=0.5 returns 200" "got status $INJECT_STATUS"
fi

# Wait for fault to appear (pipeline: config change -> sensor detects -> /diagnostics -> bridge -> fault_manager)
echo "  Waiting for LIDAR_SIM fault to appear (max 30s)..."
if poll_until "/faults" '.items[] | select(.fault_code == "LIDAR_SIM")' 30; then
    pass "LIDAR_SIM fault appeared in /faults"
else
    fail "LIDAR_SIM fault appeared in /faults" "fault not found after 30s"
fi

# Check fault detail with environment data
if api_get "/apps/diagnostic-bridge/faults/LIDAR_SIM"; then
    pass "GET fault detail returns 200"
    if echo "$RESPONSE" | jq -e '.environment_data' > /dev/null 2>&1; then
        pass "fault detail contains environment_data"
    else
        fail "fault detail contains environment_data" "field missing from response"
    fi
else
    fail "GET fault detail returns 200" "unexpected status code"
fi

section "Check-Demo Script"

# The rosbag recording finalizes duration_after_sec after confirmation, so
# poll for it rather than racing check-demo.sh against the write.
echo "  Waiting for rosbag recording to finish (max 15s)..."
if poll_until "/apps/diagnostic-bridge/bulk-data/rosbags" '.items | length > 0' 15; then
    pass "rosbag recording available before running check-demo.sh"
else
    fail "rosbag recording available before running check-demo.sh" "no rosbag after 15s"
fi

# Run check-demo.sh again while the LIDAR_SIM fault above is active: section 8
# must show the injected noise, and sections 10-12 need an active fault.
CHECK_DEMO_PLAIN=$(run_check_demo) || true

if grep -q ': null' <<< "$CHECK_DEMO_PLAIN"; then
    fail "check-demo.sh prints no null fields with a fault active" \
         "$(grep -B1 ': null' <<< "$CHECK_DEMO_PLAIN" | head -10)"
else
    pass "check-demo.sh prints no null fields with a fault active"
fi

# Section 8: exactly the parameters the list endpoint lists, each with the
# value and ROS type its own detail endpoint returns.
CONFIG_EXPECTED=""
if api_get "/apps/lidar-sim/configurations"; then
    CONFIG_EXPECTED="[]"
    while IFS= read -r cfg_id; do
        if ! api_get "/apps/lidar-sim/configurations/${cfg_id//\//%2F}"; then
            CONFIG_EXPECTED=""
            break
        fi
        CONFIG_EXPECTED=$(jq -c --argjson d "$RESPONSE" \
            '. + [{name: $d.id, value: $d.data, type: $d["x-medkit"].parameter.type}]' <<< "$CONFIG_EXPECTED")
    done < <(jq -r '.items[].id' <<< "$RESPONSE")
fi
CONFIG_PRINTED=$(check_demo_section "$CHECK_DEMO_PLAIN" 8)
if [ -z "$CONFIG_EXPECTED" ]; then
    fail "check-demo.sh section 8 lists every LiDAR configuration with its value and type" \
         "direct read of /apps/lidar-sim/configurations failed"
elif [ "$(jq -cS 'sort_by(.name)' <<< "$CONFIG_PRINTED" 2>/dev/null)" = "$(jq -cS 'sort_by(.name)' <<< "$CONFIG_EXPECTED")" ]; then
    pass "check-demo.sh section 8 lists every LiDAR configuration with its value and type"
else
    fail "check-demo.sh section 8 lists every LiDAR configuration with its value and type" \
         "$(jq -nc --argjson p "${CONFIG_PRINTED:-[]}" --argjson e "$CONFIG_EXPECTED" \
             '{missing: ($e - $p), unexpected: ($p - $e)}' 2>/dev/null || echo "section 8 is not JSON")"
fi

if grep -q "10\. Fault Detail with Environment Data" <<< "$CHECK_DEMO_PLAIN"; then
    pass "check-demo.sh runs the fault detail section for the active fault"
else
    fail "check-demo.sh runs the fault detail section for the active fault" \
         "section 10 did not run: the owning App was not resolved"
fi

if grep -q '"snapshot_count": 0' <<< "$CHECK_DEMO_PLAIN"; then
    fail "check-demo.sh fault detail shows real snapshot data" "snapshot_count is 0"
elif grep -q '"snapshot_count":' <<< "$CHECK_DEMO_PLAIN"; then
    pass "check-demo.sh fault detail shows real snapshot data"
else
    fail "check-demo.sh fault detail shows real snapshot data" "snapshot_count field missing"
fi

if grep -q '"id": "fault_LIDAR_SIM' <<< "$CHECK_DEMO_PLAIN"; then
    pass "check-demo.sh bulk-data section lists a real rosbag recording"
else
    fail "check-demo.sh bulk-data section lists a real rosbag recording" "no fault_LIDAR_SIM rosbag id found"
fi

# Cleanup: restore config + delete fault
echo "  Cleaning up: restoring config and clearing fault..."
curl -s -X PUT "${API_BASE}/apps/lidar-sim/configurations/noise_stddev" \
    -H "Content-Type: application/json" -d '{"value": 0.01}' > /dev/null || true

curl -s -X DELETE "${API_BASE}/apps/diagnostic-bridge/faults/LIDAR_SIM" > /dev/null || true

# Verify fault is cleared (poll to avoid race with fault manager processing the DELETE)
echo "  Verifying LIDAR_SIM fault cleared (max 5s)..."
elapsed=0
cleared=false
while [ $elapsed -lt 5 ]; do
    if api_get "/faults" && ! echo "$RESPONSE" | jq -e '.items[] | select(.fault_code == "LIDAR_SIM")' > /dev/null 2>&1; then
        pass "LIDAR_SIM fault cleared after cleanup"
        cleared=true
        break
    fi
    sleep 1
    elapsed=$((elapsed + 1))
done
if [ "$cleared" = false ]; then
    fail "LIDAR_SIM fault cleared after cleanup" "fault still present after 5s"
fi

section "Triggers"

assert_triggers_crud "apps" "diagnostic-bridge" "/api/v1/apps/diagnostic-bridge/faults"

section "Beacon Discovery"

# Beacon data is exposed at vendor extension endpoints:
#   /apps/{id}/x-medkit-topic-beacon  (BEACON_MODE=topic)
#   /apps/{id}/x-medkit-param-beacon  (BEACON_MODE=param)
# When BEACON_MODE=none (CI default), these endpoints return 404.
beacon_found=false
for beacon_type in topic-beacon param-beacon; do
    if api_get "/apps/lidar-sim/x-medkit-${beacon_type}"; then
        beacon_found=true
        pass "GET /apps/lidar-sim/x-medkit-${beacon_type} returns 200"
        if echo "$RESPONSE" | jq -e '.status' > /dev/null 2>&1; then
            pass "beacon response contains 'status' field"
        else
            fail "beacon response contains 'status' field" "field missing"
        fi
        if echo "$RESPONSE" | jq -e '.entity_id' > /dev/null 2>&1; then
            pass "beacon response contains 'entity_id' field"
        else
            fail "beacon response contains 'entity_id' field" "field missing"
        fi
        break
    fi
done
if [ "$beacon_found" = false ]; then
    # Not a failure - beacons are optional depending on BEACON_MODE
    echo -e "  ${BLUE}SKIP${NC} beacon not active (BEACON_MODE=none or plugin not loaded)"
fi

section "check-demo.sh: a failed fault read is not reported as no faults"

FAILED_READ_LOG=$(mktemp)
if stop_fault_manager; then
    if api_get "/faults" 503; then
        pass "setup: GET /faults answers 503 while the fault manager is stopped"
    else
        fail "setup: GET /faults answers 503 while the fault manager is stopped" "got: $(head -c 200 <<< "${RESPONSE}")"
    fi
    if GATEWAY_URL="$GATEWAY_URL" timeout 120 bash "$CHECK_DEMO_SCRIPT" < /dev/null > "$FAILED_READ_LOG" 2>&1; then
        FAILED_READ_RC=0
    else
        FAILED_READ_RC=$?
    fi
    resume_fault_manager
    if [ "$FAILED_READ_RC" -ne 0 ] && grep -q 'Could not read faults .*(HTTP 503)' "$FAILED_READ_LOG" \
        && ! grep -q 'No active faults' "$FAILED_READ_LOG"; then
        pass "check-demo.sh exits non-zero on HTTP 503 and does not claim there are no faults"
    else
        fail "check-demo.sh exits non-zero on HTTP 503 and does not claim there are no faults" \
            "rc=${FAILED_READ_RC}; output: $(sed 's/\x1b\[[0-9;]*m//g' "$FAILED_READ_LOG" | grep -vE '^ *$' | tail -n 6 | tr '\n' ';')"
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

section "check-demo.sh: a fault reported from a sub-path of an App's node"

# The anomaly detector reports as /processing/anomaly_detector/<sensor>, a
# sub-path of its node. Sections 10-12 use the first listed fault, so the
# other faults are cleared before each report.
SUBPATH_CODE="SMOKE_SUBPATH_SOURCE"
SIBLING_CODE="SMOKE_SIBLING_SOURCE"
curl -s -m 20 -X DELETE "${API_BASE}/faults" > /dev/null || true
if report_fault "$SUBPATH_CODE" "/processing/anomaly_detector/imu_sim" \
    && poll_until "/faults" ".items[0].fault_code == \"${SUBPATH_CODE}\"" 15; then
    pass "setup: ${SUBPATH_CODE} from /processing/anomaly_detector/imu_sim is the first listed fault"
    SUBPATH_PLAIN=$(run_check_demo) || true
    SUBPATH_SHOWN=$(check_demo_section "$SUBPATH_PLAIN" 10 | jq -r '.[0].code // empty' 2>/dev/null) || true
    if [ "$SUBPATH_SHOWN" = "$SUBPATH_CODE" ] \
        && grep -q "Fetching fault ${SUBPATH_CODE} on apps/anomaly-detector\.\.\." <<< "$SUBPATH_PLAIN"; then
        pass "check-demo.sh section 10 shows a sub-path fault on the App that owns the node"
    else
        fail "check-demo.sh section 10 shows a sub-path fault on the App that owns the node" \
            "section 10 code: ${SUBPATH_SHOWN:-none}; $(grep -m1 -E 'Could not map|Fetching fault' <<< "$SUBPATH_PLAIN")"
    fi
    if grep -q "=== 11\. " <<< "$SUBPATH_PLAIN" && grep -q "=== 12\. " <<< "$SUBPATH_PLAIN"; then
        pass "check-demo.sh runs sections 11 and 12 for a sub-path fault"
    else
        fail "check-demo.sh runs sections 11 and 12 for a sub-path fault" "section 11 or 12 missing"
    fi
    if grep -q ': null' <<< "$SUBPATH_PLAIN"; then
        fail "check-demo.sh prints no null fields for a sub-path fault" \
            "$(grep -B1 ': null' <<< "$SUBPATH_PLAIN" | head -10)"
    else
        pass "check-demo.sh prints no null fields for a sub-path fault"
    fi
    # The gateway lists a node's rosbags only for faults reported under the
    # exact node path, so this section may have no recording to list.
    SUBPATH_ROSBAGS_TEXT=$(check_demo_section_text "$SUBPATH_PLAIN" 12)
    if check_demo_section "$SUBPATH_PLAIN" 12 | jq -e 'length > 0' > /dev/null 2>&1 \
        || grep -q "No rosbag recordings" <<< "$SUBPATH_ROSBAGS_TEXT"; then
        pass "check-demo.sh section 12 lists rosbags or says there are none"
    else
        fail "check-demo.sh section 12 lists rosbags or says there are none" \
            "section 12: $(tr '\n' ';' <<< "$SUBPATH_ROSBAGS_TEXT" | head -c 300)"
    fi
else
    fail "setup: ${SUBPATH_CODE} from /processing/anomaly_detector/imu_sim is the first listed fault" \
        "first listed: $(curl -s -m 10 "${API_BASE}/faults" | jq -c '.items[0] | {fault_code, reporting_sources}' 2>/dev/null)"
fi

# A source that only shares a name prefix with a node is not under it.
curl -s -m 20 -X DELETE "${API_BASE}/faults" > /dev/null || true
if report_fault "$SIBLING_CODE" "/processing/anomaly_detector_extra" \
    && poll_until "/faults" ".items[0].fault_code == \"${SIBLING_CODE}\"" 15; then
    pass "setup: ${SIBLING_CODE} from /processing/anomaly_detector_extra is the first listed fault"
    SIBLING_PLAIN=$(run_check_demo) || true
    if grep -q "Could not map fault ${SIBLING_CODE} to a reporting App" <<< "$SIBLING_PLAIN" \
        && ! grep -q "=== 10\. " <<< "$SIBLING_PLAIN"; then
        pass "check-demo.sh maps no App to a source that only shares a name prefix with its node"
    else
        fail "check-demo.sh maps no App to a source that only shares a name prefix with its node" \
            "$(grep -m1 -E 'Could not map|Fetching fault' <<< "$SIBLING_PLAIN")"
    fi
else
    fail "setup: ${SIBLING_CODE} from /processing/anomaly_detector_extra is the first listed fault" \
        "first listed: $(curl -s -m 10 "${API_BASE}/faults" | jq -c '.items[0] | {fault_code, reporting_sources}' 2>/dev/null)"
fi
curl -s -m 20 -X DELETE "${API_BASE}/faults" > /dev/null || true

section "check-demo.sh waits for a sensor without a first message"

# The first run above races the linking and often starts after it. Here the
# wait is needed on every run: on a fresh gateway the LiDAR is held before
# anything reads it and resumed 2 s into the run, well inside both the
# 30 s link wait and the 5 s first-message wait.
if docker restart "${DEMO_CONTAINER}" > /dev/null 2>&1; then
    pass "setup: ${DEMO_CONTAINER} restarted"
else
    fail "setup: ${DEMO_CONTAINER} restarted" "docker restart failed"
fi
wait_for_gateway 90
if stop_lidar; then
    pass "setup: lidar_sim held right after /health"
    ( sleep 2; resume_lidar ) &
    HOLD_PID=$!
    HELD_RC=0
    HELD_TIMED=$(run_check_demo_timed "") || HELD_RC=$?
    wait "$HOLD_PID" 2>/dev/null || true
    resume_lidar
    HELD_PLAIN=$(untimed "$HELD_TIMED")
    HELD_WAIT_TEXT=$(readiness_wait_output "$HELD_TIMED")
    echo "  readiness wait $(readiness_wait_ms "$HELD_TIMED") ms:" \
        "$(grep -vE '^ *$' <<< "$HELD_WAIT_TEXT" | tr '\n' ';' | head -c 300)"
    HELD_MISSING=""
    for n in 5 6 7 8; do
        section_values_printed "$HELD_PLAIN" "$n" || HELD_MISSING="${HELD_MISSING} ${n}"
    done
    if [ "$HELD_RC" -eq 0 ] && [ -z "$HELD_MISSING" ] && ! grep -q ': null' <<< "$HELD_PLAIN"; then
        pass "check-demo.sh with the LiDAR held exits 0 and prints values in sections 5-8"
    else
        fail "check-demo.sh with the LiDAR held exits 0 and prints values in sections 5-8" \
            "exit code ${HELD_RC}; sections without values:${HELD_MISSING:- none}; last lines: $(grep -vE '^ *$' <<< "$HELD_PLAIN" | tail -n 3 | tr '\n' ';')"
    fi
    if grep -q "^Waiting for .*lidar-sim" <<< "$HELD_WAIT_TEXT"; then
        pass "check-demo.sh says it waits for the held LiDAR"
    else
        fail "check-demo.sh says it waits for the held LiDAR" \
            "wait printed: $(grep -vE '^ *$' <<< "$HELD_WAIT_TEXT" | tr '\n' ';' | head -c 300)"
    fi
else
    fail "setup: lidar_sim held right after /health" "no lidar_sim_node process in ${DEMO_CONTAINER}"
fi

section "check-demo.sh with a failed IMU on a fresh gateway"

# The gateway keeps the last message of a topic it has read. A sensor that
# stops before its first read has no message at all, so the demo restarts
# and the IMU fails before anything reads it.
if docker restart "${DEMO_CONTAINER}" > /dev/null 2>&1; then
    pass "setup: ${DEMO_CONTAINER} restarted"
else
    fail "setup: ${DEMO_CONTAINER} restarted" "docker restart failed"
fi
wait_for_gateway 90
wait_for_runtime_linking "/apps/imu-sim/data" 60
assert_script_execution "compute-unit" "inject-failure" 30
if poll_until "/faults" '.items | length > 0' 30; then
    pass "setup: a fault is listed after the IMU failure"
else
    fail "setup: a fault is listed after the IMU failure" "no fault after 30 s"
fi
if api_get "/health" && jq -e '."x-medkit-data-provider".pool_size == 0' <<< "$RESPONSE" > /dev/null 2>&1; then
    pass "setup: the gateway has read no topic before check-demo.sh runs"
else
    fail "setup: the gateway has read no topic before check-demo.sh runs" \
        "$(jq -c '."x-medkit-data-provider"' <<< "$RESPONSE" 2>/dev/null)"
fi

# check-demo.sh bounds each request of its readiness wait to this many seconds.
CHECK_DEMO_REQUEST_TIMEOUT_SEC=3

FAILED_IMU_RC=0
FAILED_IMU_TIMED=$(run_check_demo_timed "") || FAILED_IMU_RC=$?
FAILED_IMU_PLAIN=$(untimed "$FAILED_IMU_TIMED")
FAILED_IMU_WAIT_MS=$(readiness_wait_ms "$FAILED_IMU_TIMED")
echo "  readiness wait ${FAILED_IMU_WAIT_MS} ms, exit code ${FAILED_IMU_RC}"

if [ "$FAILED_IMU_RC" -eq 0 ]; then
    pass "check-demo.sh exits 0 with the IMU failed"
else
    fail "check-demo.sh exits 0 with the IMU failed" \
        "exit code ${FAILED_IMU_RC}: $(grep -vE '^ *$' <<< "$FAILED_IMU_PLAIN" | tail -n 3 | tr '\n' ';')"
fi
if grep -q ': null' <<< "$FAILED_IMU_PLAIN"; then
    fail "check-demo.sh prints no null fields with the IMU failed" \
        "$(grep -B1 ': null' <<< "$FAILED_IMU_PLAIN" | head -10)"
else
    pass "check-demo.sh prints no null fields with the IMU failed"
fi
FAILED_IMU_SECTION_6=$(check_demo_section_text "$FAILED_IMU_PLAIN" 6)
if grep -q "=== 6\. " <<< "$FAILED_IMU_PLAIN" \
    && check_demo_section "$FAILED_IMU_PLAIN" 6 | jq -e 'length == 0' > /dev/null 2>&1 \
    && grep -qi "no IMU data" <<< "$FAILED_IMU_SECTION_6"; then
    pass "check-demo.sh section 6 says the IMU has no data"
else
    fail "check-demo.sh section 6 says the IMU has no data" \
        "section 6: $(tr '\n' ';' <<< "$FAILED_IMU_SECTION_6" | head -c 300)"
fi
if section_values_printed "$FAILED_IMU_PLAIN" 5 && section_values_printed "$FAILED_IMU_PLAIN" 7; then
    pass "check-demo.sh prints LiDAR and GPS values with the IMU failed"
else
    fail "check-demo.sh prints LiDAR and GPS values with the IMU failed" \
        "section 5: $(check_demo_section "$FAILED_IMU_PLAIN" 5 | jq -c . 2>/dev/null | head -c 150); section 7: $(check_demo_section "$FAILED_IMU_PLAIN" 7 | jq -c . 2>/dev/null | head -c 150)"
fi
FAILED_IMU_MISSING=""
for n in 9 10 11 12; do
    grep -q "=== ${n}\. " <<< "$FAILED_IMU_PLAIN" || FAILED_IMU_MISSING="${FAILED_IMU_MISSING} ${n}"
done
if [ -z "$FAILED_IMU_MISSING" ]; then
    pass "check-demo.sh runs sections 9-12 with the IMU failed"
else
    fail "check-demo.sh runs sections 9-12 with the IMU failed" "missing sections:${FAILED_IMU_MISSING}"
fi

# The wait names only the sensor it waited for; LiDAR and GPS had data.
FAILED_IMU_WAIT_TEXT=$(readiness_wait_output "$FAILED_IMU_TIMED")
if grep -q "imu-sim" <<< "$FAILED_IMU_WAIT_TEXT" && ! grep -qE "lidar-sim|gps-sim" <<< "$FAILED_IMU_WAIT_TEXT"; then
    pass "check-demo.sh's wait names the sensor it waited for"
else
    fail "check-demo.sh's wait names the sensor it waited for" \
        "wait printed: $(grep -vE '^ *$' <<< "$FAILED_IMU_WAIT_TEXT" | tr '\n' ';' | head -c 300)"
fi

# The wait is bounded by wall-clock time: DATA_WAIT_SEC plus one request.
# A cold IMU read blocks for the gateway's sample timeout, so a wait that
# counts passes instead of seconds overruns here.
if [ "$FAILED_IMU_WAIT_MS" -le $(( (30 + CHECK_DEMO_REQUEST_TIMEOUT_SEC) * 1000 )) ]; then
    pass "check-demo.sh's readiness wait ends within the default 30 s plus one request"
else
    fail "check-demo.sh's readiness wait ends within the default 30 s plus one request" \
        "waited ${FAILED_IMU_WAIT_MS} ms"
fi
# 08 is a whole number of seconds with a leading zero, not an octal number.
# Here the IMU's 5 s first-message window ends the wait before 8 s, so the
# wait must last at least 3 s: a value read as 0 or refused ends it at once.
for wait_sec in 0 2 08; do
    WAIT_RC=0
    WAIT_TIMED=$(run_check_demo_timed "$wait_sec") || WAIT_RC=$?
    WAIT_MS=$(readiness_wait_ms "$WAIT_TIMED")
    WAIT_MIN_MS=0
    [ "$wait_sec" = 08 ] && WAIT_MIN_MS=3000
    echo "  DATA_WAIT_SEC=${wait_sec}: readiness wait ${WAIT_MS} ms, exit code ${WAIT_RC}"
    if [ "$WAIT_RC" -eq 0 ] && [ "$WAIT_MS" -ge "$WAIT_MIN_MS" ] \
        && [ "$WAIT_MS" -le $(( (10#$wait_sec + CHECK_DEMO_REQUEST_TIMEOUT_SEC) * 1000 )) ] \
        && grep -q "=== 12\. " <<< "$WAIT_TIMED"; then
        pass "check-demo.sh with DATA_WAIT_SEC=${wait_sec} waits at most ${wait_sec} s plus one request and runs to section 12"
    else
        fail "check-demo.sh with DATA_WAIT_SEC=${wait_sec} waits at most ${wait_sec} s plus one request and runs to section 12" \
            "exit code ${WAIT_RC}, waited ${WAIT_MS} ms: $(grep -vE '^[0-9]+ *$' <<< "$WAIT_TIMED" | tail -n 2 | tr '\n' ';')"
    fi
done

assert_script_execution "compute-unit" "restore-normal" 30

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
