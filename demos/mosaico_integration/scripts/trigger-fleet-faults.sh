#!/bin/bash
# Fleet fault injection - three LIDAR_SIM faults, one under rotation.
#
# Scenario: all three robots report LIDAR_SIM (Step 1 metadata filter
# matches 3 of 3). Robot-02 is rotating during the fault window, so
# the compound IMU .Q stationarity query (Step 2) excludes it - leaving
# 2 of 3 sequences for content-level drill-down (noise vs drift).
#
#   robot-01 warehouse-A  : LIDAR noise_stddev=0.5 , stationary
#   robot-02 warehouse-B  : LIDAR noise_stddev=0.5 , rotating (IMU drift_rate=0.3 rad/s)
#   robot-03 outdoor-yard : LIDAR drift_rate=0.5   , stationary
#
# The medkit side of robot-02 will also emit an IMU DRIFTING diagnostic
# once |drift_offset| > 0.1 rad, which becomes a second fault on the
# gateway. The bridge's FAULT_CODE_ALLOWLIST=LIDAR_SIM env var (set in
# docker-compose.fleet.yml) keeps that IMU fault out of Mosaico so the
# ingested catalog stays at one LIDAR_SIM sequence per robot.
set -euo pipefail

IMU_DRIFT_RAD_PER_S="${IMU_DRIFT_RAD_PER_S:-0.3}"
PRE_FAULT_WAIT_SEC="${PRE_FAULT_WAIT_SEC:-20}"

# put_config <robot_id> <host:port> <app> <param> <json-value>
# Wraps the PUT so a curl failure names the robot + endpoint instead of
# surfacing a bare `curl: (22) ...` via set -e.
put_config() {
    local robot="$1" addr="$2" app="$3" param="$4" value="$5"
    if ! curl -sf -X PUT "http://${addr}/api/v1/apps/${app}/configurations/${param}" \
            -H "Content-Type: application/json" -d "{\"value\": ${value}}" > /dev/null; then
        echo "!! ${robot}: PUT ${app}/${param}=${value} FAILED at ${addr}" >&2
        exit 1
    fi
}

echo "=== Fleet Fault Injection ==="
echo ""

echo ">> robot-02 (warehouse-B): IMU drift_rate -> ${IMU_DRIFT_RAD_PER_S} rad/s (rotating)"
put_config robot-02 localhost:18082 imu-sim   drift_rate    "${IMU_DRIFT_RAD_PER_S}"

echo ">> Priming the ring buffer with ${PRE_FAULT_WAIT_SEC}s of rotation data..."
sleep "${PRE_FAULT_WAIT_SEC}"

echo ">> robot-01 (warehouse-A): LIDAR noise_stddev -> 0.5"
put_config robot-01 localhost:18081 lidar-sim noise_stddev  0.5

echo ">> robot-02 (warehouse-B): LIDAR noise_stddev -> 0.5 (same as robot-01, but under rotation)"
put_config robot-02 localhost:18082 lidar-sim noise_stddev  0.5

echo ">> robot-03 (outdoor-yard): LIDAR drift_rate -> 0.5"
put_config robot-03 localhost:18083 lidar-sim drift_rate    0.5

echo ""
echo "=== Injected ==="
echo "   robot-01: LIDAR noise                (stationary)"
echo "   robot-02: LIDAR noise under rotation (IMU angular_velocity.z ~ ${IMU_DRIFT_RAD_PER_S} rad/s)"
echo "   robot-03: LIDAR drift                (stationary)"
echo ""
echo "Wait ~60s for post-fault capture + bridge ingest."
echo ""
echo "Expected in Mosaico after ingest:"
echo "   3 sequences, all fault_code=LIDAR_SIM"
echo "   compound IMU.Q stationarity query matches 2 of 3 (robot-02 excluded by rotation)"
