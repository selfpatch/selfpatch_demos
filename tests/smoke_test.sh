#!/bin/bash
# Smoke tests for sensor_diagnostics demo
# Runs from the host against the containerized gateway on localhost:8080
#
# Usage: ./tests/smoke_test.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

CHECK_DEMO_SCRIPT="${SCRIPT_DIR}/../demos/sensor_diagnostics/check-demo.sh"

# Runs check-demo.sh and prints its output without ANSI colours.
run_check_demo() {
    GATEWAY_URL="$GATEWAY_URL" bash "$CHECK_DEMO_SCRIPT" 2>&1 | sed 's/\x1b\[[0-9;]*m//g'
}

# Prints the JSON values printed in section N of a check-demo.sh output as
# one array: [] when the section is missing, nothing when it is not JSON.
# Usage: check_demo_section OUTPUT N
check_demo_section() {
    awk -v hdr="^=== $2\\\\. " '
        $0 ~ hdr { on = 1; next }
        on && /^=== [0-9]+\. / { exit }
        on' <<< "$1" | sed -n '/^[[{]/,$p' | jq -s '.' 2>/dev/null || true
}

# Prints the value of a configuration, or nothing when it cannot be read.
# Usage: config_value APP PARAM
config_value() {
    curl -s -m 10 "${API_BASE}/apps/$1/configurations/$2" | jq -c '.data' 2>/dev/null || true
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
# check-demo.sh must then wait for real values, or stop with a message and a
# non-zero exit, and never print null.
EARLY_LIDAR_ITEMS=$(curl -s -m 5 "${API_BASE}/apps/lidar-sim/data" | jq '.items | length' 2>/dev/null) || true
EARLY_RC=0
EARLY_PLAIN=$(run_check_demo) || EARLY_RC=$?
echo "  LiDAR data items when check-demo.sh started: ${EARLY_LIDAR_ITEMS:-unreadable}; exit code ${EARLY_RC}"

if grep -q ': null' <<< "$EARLY_PLAIN"; then
    fail "check-demo.sh started right after /health prints no null fields" \
         "$(grep -B1 ': null' <<< "$EARLY_PLAIN" | head -10)"
else
    pass "check-demo.sh started right after /health prints no null fields"
fi

# True when sections 5-8 of the early run carry values of the right type.
early_values_printed() {
    check_demo_section "$EARLY_PLAIN" 5 | jq -e 'length == 1 and ([.[0]
        | .angle_min, .angle_max, .range_min, .range_max, .sample_ranges[]]
        | length == 9 and all(type == "number"))' > /dev/null 2>&1 &&
    check_demo_section "$EARLY_PLAIN" 6 | jq -e 'length == 1 and ([.[0]
        | .linear_acceleration[], .angular_velocity[]]
        | length == 6 and all(type == "number"))' > /dev/null 2>&1 &&
    check_demo_section "$EARLY_PLAIN" 7 | jq -e 'length == 1 and ([.[0]
        | .latitude, .longitude, .altitude] | all(type == "number"))' > /dev/null 2>&1 &&
    check_demo_section "$EARLY_PLAIN" 8 | jq -e 'length > 0' > /dev/null 2>&1
}

if [ "$EARLY_RC" -ne 0 ]; then
    if grep -q "Sensor data not available" <<< "$EARLY_PLAIN"; then
        pass "check-demo.sh started right after /health prints sensor values or stops with a message"
    else
        fail "check-demo.sh started right after /health prints sensor values or stops with a message" \
             "exit code ${EARLY_RC} without the message: $(tail -3 <<< "$EARLY_PLAIN")"
    fi
elif early_values_printed; then
    pass "check-demo.sh started right after /health prints sensor values or stops with a message"
else
    fail "check-demo.sh started right after /health prints sensor values or stops with a message" \
         "exit code 0 but sections 5-8 lack values"
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

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
