#!/bin/bash
# Restore normal operation: cancel goals, restore velocity params, put AMCL back on
# the robot's pose in the simulation, clear all faults
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

# A failed step is named on stderr, which the Scripts API returns as the error
# message. The remaining steps still run; the script fails at the end.
FAILURES=0
report_failure() {
    echo "FAIL: $1" >&2
    FAILURES=$((FAILURES + 1))
}

# Sets one parameter and reports a refused or failed write.
# Usage: put_config APP PARAM JSON_VALUE
put_config() {
    local app="$1" param="$2" value="$3" response code
    response=$(curl -s -m 30 -w "\n%{http_code}" -X PUT \
        "${API_BASE}/apps/${app}/configurations/${param}" \
        -H "Content-Type: application/json" -d "{\"value\": ${value}}" 2>/dev/null) || response=$'\nnone'
    code=$(tail -n 1 <<< "${response}")
    case "${code}" in
        2??) echo "  ${app}/${param} = ${value}" ;;
        *) report_failure "${app}/${param} (HTTP ${code}: $(sed '$d' <<< "${response}" | jq -r '.message // empty' 2>/dev/null))" ;;
    esac
}

echo "Canceling active navigation goals..."
EXECUTIONS=$(curl -s "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" 2>/dev/null || echo '{"items":[]}')
if echo "${EXECUTIONS}" | jq -e '.items[]' > /dev/null 2>&1; then
    echo "${EXECUTIONS}" | jq -r '.items[].id' | while read -r EXEC_ID; do
        if [ -n "${EXEC_ID}" ] && [[ "${EXEC_ID}" =~ ^[a-zA-Z0-9_-]+$ ]]; then
            curl -s -X DELETE "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/${EXEC_ID}" > /dev/null 2>&1 || true
            echo "  Canceled execution: ${EXEC_ID}"
        fi
    done
else
    echo "  No active executions found."
fi

echo ""
echo "Restoring velocity parameters to defaults..."
put_config velocity-smoother max_velocity "[0.26, 0.0, 1.0]"
put_config controller-server FollowPath.max_vel_x 0.26

# The map frame of this demo is the Gazebo world frame, so the robot's world pose
# is the pose AMCL must hold. It is read from the running simulation after the
# robot has stopped, never assumed.
echo ""
echo "Re-localizing AMCL at the robot's pose in the simulation..."
sleep 1
MODEL="${TURTLEBOT3_MODEL:-burger}"
WORLD=$(timeout 10 gz topic -l 2>/dev/null | sed -n 's|^/world/\([^/]*\)/dynamic_pose/info$|\1|p' | head -n 1) || WORLD=""
POSE_REQUEST=""
if [ -n "${WORLD}" ]; then
    # A single read has returned the model twice, so only the first match is used.
    POSE_REQUEST=$(timeout 10 gz topic -e -n 1 -t "/world/${WORLD}/dynamic_pose/info" --json-output 2>/dev/null \
        | jq -c -n --arg m "${MODEL}" '
            first(inputs | .pose[] | select(.name == $m))
            | (.orientation | [(.w // 0), (.x // 0), (.y // 0), (.z // 0)] as [$w, $x, $y, $z]
               | atan2(2 * ($w * $z + $x * $y); 1 - 2 * ($y * $y + $z * $z))) as $yaw
            | {parameters: {pose: {header: {frame_id: "map"},
                pose: {pose: {position: {x: (.position.x // 0), y: (.position.y // 0), z: 0.0},
                              orientation: {x: 0.0, y: 0.0, z: ($yaw / 2 | sin), w: ($yaw / 2 | cos)}},
                       covariance: [range(36) | if . == 0 or . == 7 or . == 35 then 0.01 else 0.0 end]}}}}' \
        2>/dev/null) || POSE_REQUEST=""
fi

if [ -z "${POSE_REQUEST}" ]; then
    report_failure "amcl/set_initial_pose (no pose for '${MODEL}' in the simulation)"
else
    echo "  Robot pose: $(echo "${POSE_REQUEST}" | jq -c '.parameters.pose.pose.pose.position | {x, y}')"
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X POST \
        "${API_BASE}/apps/amcl/operations/set_initial_pose/executions" \
        -H "Content-Type: application/json" -d "${POSE_REQUEST}" 2>/dev/null) || HTTP_CODE="none"
    if [ "${HTTP_CODE}" != "200" ]; then
        report_failure "amcl/set_initial_pose (HTTP ${HTTP_CODE})"
    fi
fi

# Faults are cleared after the re-localization, so no pose from the scattered
# particle cloud is reported after the clear.
echo ""
echo "Clearing all faults..."
CLEAR_CODE=$(curl -s -o /dev/null -w "%{http_code}" -X DELETE "${API_BASE}/faults" 2>/dev/null) || CLEAR_CODE="none"
case "${CLEAR_CODE}" in
    2??) ;;
    *) report_failure "faults clear (HTTP ${CLEAR_CODE})" ;;
esac

FAULT_COUNT=$(curl -sf "${API_BASE}/faults" | jq '.items | length' 2>/dev/null || echo "?")
if [ "${FAILURES}" -gt 0 ]; then
    echo ""
    echo "Normal operation is not restored: ${FAILURES} step(s) failed. Active faults: ${FAULT_COUNT}"
    echo "${FAILURES} restore step(s) failed" >&2
    exit 1
fi

echo ""
echo "Normal operation restored."
echo "Active faults: ${FAULT_COUNT}"
exit 0
