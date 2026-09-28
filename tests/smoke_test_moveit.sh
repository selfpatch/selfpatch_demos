#!/bin/bash
# Smoke tests for moveit_pick_place demo
# Runs from the host against the containerized gateway on localhost:8080
#
# Tests: health, entity discovery (areas/components/apps/functions),
#   discovery relationships, Linux introspection, data access, operations,
#   configurations, scripts (list + execution), bulk data, faults, logs,
#   trigger CRUD lifecycle
# Uses demo.launch.py (fake hardware, no Gazebo) for CI stability
#
# Usage: ./tests/smoke_test_moveit.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

DEMO_DIR="$(cd "${SCRIPT_DIR}/../demos/moveit_pick_place" && pwd)"
DEMO_CONTAINER="${MOVEIT_DEMO_CONTAINER:-moveit_medkit_demo_ci}"
ACTION="/panda_arm_controller/follow_joint_trajectory"
PICK_PLACE_PATTERN='^python3 /root/demo_ws/install/moveit_medkit_demo/lib/moveit_medkit_demo/pick_place_loop\.py'

# container_python <args...>: run the Python script on stdin inside the demo
# container, with the ROS 2 environment sourced. The process is killed after
# 240 s, which is longer than every wait inside CONTROLLER_PROBE_PY together.
container_python() {
    docker exec -i "${DEMO_CONTAINER}" bash -c '
        set +u
        source /opt/ros/jazzy/setup.bash
        source /root/demo_ws/install/setup.bash
        exec timeout 240 python3 - "$@"
    ' container_python "$@"
}

# Arm controller probe, run with container_python. Every mode first waits up
# to 30 s for the action server, then up to 60 s until neither the arm
# controller nor MoveGroup has an active goal for 1 s. Exits non-zero when a
# wait runs out.
#   idle <action>:    only these waits.
#   fault <action>:   send two goals back to back. The second preempts the
#                     first, which manipulation_monitor reports as a fault.
#   preempt <action>: print ARMED, then preempt the next goal the arm
#                     controller starts, and print that goal's final status.
CONTROLLER_PROBE_PY=$(cat <<'PY'
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from trajectory_msgs.msg import JointTrajectoryPoint

mode, arm_action = sys.argv[1], sys.argv[2]
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING, GoalStatus.STATUS_CANCELING)
TERMINAL = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_CANCELED: "CANCELED",
            GoalStatus.STATUS_ABORTED: "ABORTED"}
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

rclpy.init()
node = Node("smoke_controller_probe")
goals = {}
subscriptions = []
for action in (arm_action, "/move_action"):
    goals[action] = []
    subscriptions.append(node.create_subscription(
        GoalStatusArray, action + "/_action/status",
        lambda msg, action=action: goals.__setitem__(action, list(msg.status_list)),
        qos_profile_action_status_default))
client = ActionClient(node, FollowJointTrajectory, arm_action)


def spin_until(condition, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return True
    return False


def wait_future(future, timeout, what):
    if not spin_until(future.done, timeout):
        sys.exit(f"no {what} within {timeout} s")
    return future.result()


def send_ready_goal(sec):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f"panda_joint{i}" for i in range(1, 8)]
    goal.trajectory.points = [JointTrajectoryPoint(positions=READY, time_from_start=Duration(sec=sec))]
    handle = wait_future(client.send_goal_async(goal), 10, "goal response")
    if not handle.accepted:
        sys.exit("arm controller rejected the goal")
    return handle


def final_status(handle):
    return TERMINAL.get(wait_future(handle.get_result_async(), 10, "goal result").status, "UNKNOWN")


quiet_since = None


def idle():
    global quiet_since
    busy = any(s.status in ACTIVE for status_list in goals.values() for s in status_list)
    matched = all(sub.get_publisher_count() > 0 for sub in subscriptions)
    if busy or not matched:
        quiet_since = None
        return False
    quiet_since = quiet_since or time.monotonic()
    return time.monotonic() - quiet_since >= 1.0


if not client.wait_for_server(timeout_sec=30):
    sys.exit("arm controller action server not available within 30 s")
if not spin_until(idle, 60):
    sys.exit("arm controller or MoveGroup still busy after 60 s")

if mode == "fault":
    first = send_ready_goal(3)
    second = send_ready_goal(1)
    first_status, second_status = final_status(first), final_status(second)
    print(f"first goal {first_status}, second goal {second_status}", flush=True)
    if first_status == "SUCCEEDED":
        sys.exit("the second goal did not preempt the first")

if mode == "preempt":
    seen = {bytes(s.goal_info.goal_id.uuid) for s in goals[arm_action]}
    print("ARMED", flush=True)
    target = []

    def new_goal_executing():
        target[:] = [bytes(s.goal_info.goal_id.uuid) for s in goals[arm_action]
                     if s.status == GoalStatus.STATUS_EXECUTING
                     and bytes(s.goal_info.goal_id.uuid) not in seen][:1]
        return bool(target)

    if not spin_until(new_goal_executing, 90):
        sys.exit("no new arm goal started within 90 s")
    competing = send_ready_goal(1)
    target_status = []

    def target_finished():
        target_status[:] = [TERMINAL[s.status] for s in goals[arm_action]
                            if bytes(s.goal_info.goal_id.uuid) == target[0] and s.status in TERMINAL]
        return bool(target_status)

    if not spin_until(target_finished, 10):
        sys.exit("the goal to preempt did not finish within 10 s")
    print(f"PREEMPTED goal {target_status[0]}, competing goal {final_status(competing)}", flush=True)
    if target_status[0] == "SUCCEEDED":
        sys.exit("the competing goal did not preempt the running goal")

node.destroy_node()
rclpy.shutdown()
PY
)

# pick_place_loop sends MoveGroup goals to the arm controller that move-arm.sh
# drives. SIGSTOP pauses it without changing the demo. A MoveGroup goal it
# already sent still runs to its end, so wait until the arm is idle.
stop_pick_place_loop() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${PICK_PLACE_PATTERN}') || exit 0
        kill -STOP \"\${pid}\"
    " > /dev/null 2>&1 || true
    if ! container_python idle "${ACTION}" <<< "${CONTROLLER_PROBE_PY}" \
        > /tmp/moveit_smoke_idle.log 2>&1; then
        fail "setup: arm idle after pausing pick_place_loop" "$(tail -n 3 /tmp/moveit_smoke_idle.log)"
    fi
}

resume_pick_place_loop() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${PICK_PLACE_PATTERN}') || exit 0
        kill -CONT \"\${pid}\"
    " > /dev/null 2>&1 || true
}

# start_preemptor <log>: start the preempt probe in the background and return
# once it listens. Sets PREEMPTOR_PID.
start_preemptor() {
    local log="$1" waited=0
    # Empty the log first: an ARMED line left by an earlier run must not count.
    : > "${log}"
    container_python preempt "${ACTION}" <<< "${CONTROLLER_PROBE_PY}" >> "${log}" 2>&1 &
    PREEMPTOR_PID=$!
    until grep -q '^ARMED' "${log}"; do
        if ! kill -0 "${PREEMPTOR_PID}" 2> /dev/null || [ "${waited}" -ge 900 ]; then
            return 1
        fi
        sleep 0.1
        waited=$((waited + 1))
    done
}

# finish_preemptor <log>: wait for the preempt probe. It must have preempted a
# running goal. Sets PREEMPTED_GOAL_STATUS to that goal's final status.
finish_preemptor() {
    local log="$1" rc=0
    wait "${PREEMPTOR_PID}" || rc=$?
    PREEMPTED_GOAL_STATUS=$(sed -nE 's/^PREEMPTED goal ([A-Z]+),.*/\1/p' "${log}")
    if [ "${rc}" -eq 0 ] && [ -n "${PREEMPTED_GOAL_STATUS}" ] && [ "${PREEMPTED_GOAL_STATUS}" != "SUCCEEDED" ]; then
        pass "setup: the competing goal preempted a running goal (${PREEMPTED_GOAL_STATUS})"
    else
        fail "setup: the competing goal preempted a running goal" \
            "probe rc=${rc}: $(tail -n 2 "${log}" | tr '\n' ';')"
    fi
}

# Upper bound for one move-arm.sh run. The script itself allows 3 x 30 s per
# goal, and the demo cycle runs three goals.
MOVE_ARM_LIMIT_SEC=400

# run_move_arm <log> <args...>: move-arm.sh without a TTY. Sets MOVE_ARM_RC.
run_move_arm() {
    local log="$1"
    shift
    if CONTAINER_NAME="${DEMO_CONTAINER}" timeout "${MOVE_ARM_LIMIT_SEC}" \
        "${DEMO_DIR}/move-arm.sh" "$@" < /dev/null > "${log}" 2>&1; then
        MOVE_ARM_RC=0
    else
        MOVE_ARM_RC=$?
    fi
    if [ "${MOVE_ARM_RC}" -eq 124 ]; then
        fail "move-arm.sh $* finishes within ${MOVE_ARM_LIMIT_SEC} s" "killed by timeout"
    fi
}

# real_status <log>: the final status the action client printed itself, read
# without move-arm.sh's own verdict. Empty when no goal finished.
real_status() {
    { grep -F 'Goal finished with status:' "$1" || true; } | tail -n 1 | sed -E 's/.*status: *//'
}

# reports_status <file> <label> <status>: the file holds exactly one result
# line. It is the success line for <label> when <status> is SUCCEEDED, else a
# failure line for <label> that names <status>. An empty <status> never
# matches: every goal the tests send must reach a final status.
reports_status() {
    local file="$1" label="$2" status="$3" results
    [ -n "${status}" ] || return 1
    results=$(grep -cE '^(✅ Done|Failed):' "${file}" || true)
    [ "${results}" -eq 1 ] || return 1
    if [ "${status}" = "SUCCEEDED" ]; then
        grep -qFx "✅ Done: ${label}" "${file}"
    else
        grep -qFx "Failed: ${label} (status: ${status})" "${file}"
    fi
}

# check_goal_report <log> <rc> <label>: the goal reached a final status. A real
# SUCCEEDED exits 0 with the success line; any other real status exits
# non-zero with a failure line.
check_goal_report() {
    local log="$1" rc="$2" label="$3" status
    status=$(real_status "${log}")
    if [ -z "${status}" ]; then
        fail "move-arm.sh ${label}: the action reports a final status" \
            "no 'Goal finished with status' line; rc=${rc}; tail: $(tail -n 3 "${log}" | tr '\n' ';')"
    elif [ "${status}" = "SUCCEEDED" ]; then
        if [ "${rc}" -eq 0 ] && reports_status "${log}" "${label}" "${status}"; then
            pass "move-arm.sh ${label}: real SUCCEEDED exits 0 with a success line"
        else
            fail "move-arm.sh ${label}: real SUCCEEDED exits 0 with a success line" \
                "rc=${rc}; result lines: $(grep -E '^(✅ Done|Failed):' "${log}" | tr '\n' ';')"
        fi
    else
        if [ "${rc}" -ne 0 ] && reports_status "${log}" "${label}" "${status}"; then
            pass "move-arm.sh ${label}: real ${status} exits non-zero with a failure line"
        else
            fail "move-arm.sh ${label}: real ${status} exits non-zero with a failure line" \
                "rc=${rc}; result lines: $(grep -E '^(✅ Done|Failed):' "${log}" | tr '\n' ';')"
        fi
    fi
}

# print_summary reads the script's exit status from $?, so hand it the
# status saved on entry. set +e: under errexit `(exit rc)` would end the
# trap before print_summary runs.
cleanup_on_exit() {
    local rc=$?
    set +e
    resume_pick_place_loop
    (exit "${rc}")
    print_summary
}
trap cleanup_on_exit EXIT

# --- Wait for gateway startup ---

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

test_entity_discovery "areas" manipulation planning diagnostics bridge
test_entity_discovery "components" panda-arm panda-gripper moveit-planning pick-place-loop gateway fault-manager diagnostic-bridge
test_entity_discovery "apps" joint-state-broadcaster panda-arm-controller panda-hand-controller robot-state-publisher move-group pick-place-node medkit-gateway medkit-fault-manager diagnostic-bridge-app manipulation-monitor
test_entity_discovery "functions" pick-and-place motion-planning gripper-control fault-management

section "Discovery Relationships"

assert_non_empty_items "/areas/manipulation/components"

section "Linux Introspection"

assert_procfs_introspection "medkit-gateway"

section "Data Access"

assert_non_empty_items "/apps/medkit-gateway/data"

section "Operations"

# fault_manager services may take extra time to be discovered via runtime graph introspection
echo "  Waiting for fault-manager operations to appear (max 30s)..."
if poll_until "/apps/medkit-fault-manager/operations" '.items | length > 0' 30; then
    pass "GET /apps/medkit-fault-manager/operations returns non-empty items"
else
    fail "GET /apps/medkit-fault-manager/operations returns non-empty items" "items still empty after 30s"
fi

section "Configurations"

assert_non_empty_items "/apps/medkit-gateway/configurations"

section "Scripts"

assert_scripts_list "moveit-planning" "arm-self-test"
assert_script_execution "moveit-planning" "arm-self-test" 30

section "Bulk Data"

if api_get "/apps/diagnostic-bridge-app/bulk-data"; then
    pass "GET /apps/diagnostic-bridge-app/bulk-data returns 200"
else
    fail "GET /apps/diagnostic-bridge-app/bulk-data returns 200" "unexpected status code"
fi

section "Faults"

if api_get "/faults"; then
    pass "GET /faults returns 200"
else
    fail "GET /faults returns 200" "unexpected status code"
fi

section "Logs"

assert_non_empty_items "/apps/medkit-gateway/logs"

section "Triggers"

assert_triggers_crud "apps" "diagnostic-bridge-app" "/api/v1/apps/diagnostic-bridge-app/faults"

section "check-entities.sh: real values, including under an active fault"

# Force a real fault: two goals back to back on the arm controller. The second
# preempts the first, a real ABORTED goal that manipulation_monitor reports as
# TRAJECTORY_EXECUTION_FAILED / CONTROLLER_TIMEOUT.
echo "  Forcing a real active fault via controller goal preemption..."
stop_pick_place_loop
if container_python fault "${ACTION}" <<< "${CONTROLLER_PROBE_PY}" > /tmp/moveit_smoke_fault_setup.log 2>&1; then
    pass "setup: a second goal preempted the first ($(tail -n 1 /tmp/moveit_smoke_fault_setup.log))"
else
    fail "setup: a second goal preempted the first" "$(tail -n 2 /tmp/moveit_smoke_fault_setup.log | tr '\n' ';')"
fi
resume_pick_place_loop

echo "  Waiting for the fault to appear (max 20s)..."
if poll_until "/faults" '[.items[] | select(.reporting_sources | index("/bridge/manipulation_monitor"))] | length > 0' 20; then
    pass "setup: manipulation_monitor reports a real active fault"
else
    fail "setup: manipulation_monitor reports a real active fault" "no such fault after forced controller preemption"
fi

ENTITIES_LOG=/tmp/moveit_smoke_entities.log
FAULT_FIELDS='[.items[] | {fault_code, severity_label, status, reporting_sources}] | sort_by(.fault_code)'

# Faults can change state while the script runs. Read them on both sides of
# the run and retry until both reads agree.
for attempt in 1 2 3; do
    API_FAULTS=$(curl -s -m 30 "${API_BASE}/faults")
    GATEWAY_URL="${GATEWAY_URL}" timeout 120 "${DEMO_DIR}/check-entities.sh" < /dev/null 2>&1 \
        | sed 's/\x1b\[[0-9;]*m//g' > "${ENTITIES_LOG}" || true
    API_FAULTS_AFTER=$(curl -s -m 30 "${API_BASE}/faults")
    if [ "$(jq -c "${FAULT_FIELDS}" <<< "${API_FAULTS}")" = "$(jq -c "${FAULT_FIELDS}" <<< "${API_FAULTS_AFTER}")" ]; then
        break
    fi
    echo "  Faults changed during attempt ${attempt}/3, running again..."
done

if ! grep -q 'exploration complete' "${ENTITIES_LOG}"; then
    fail "check-entities.sh runs to completion" "$(tail -n 5 "${ENTITIES_LOG}")"
elif grep -q '": null' "${ENTITIES_LOG}"; then
    fail "check-entities.sh prints no null fields" \
        "$(grep '": null' "${ENTITIES_LOG}" | sort -u | tr '\n' ';')"
else
    pass "check-entities.sh prints no null fields"
fi

# shown_items <n>: the JSON objects check-entities.sh printed under its
# "=== <n>. ..." heading, as one array.
shown_items() {
    awk -v heading="=== $1. " '
        index($0, "=== ") == 1 { on = (index($0, heading) == 1); next }
        index($0, "Entity hierarchy exploration complete") { on = 0 }
        on
    ' "${ENTITIES_LOG}" | jq -s -c '.'
}

SHOWN_COMPONENTS=$(shown_items 2) || SHOWN_COMPONENTS='[]'
SHOWN_APPS=$(shown_items 3) || SHOWN_APPS='[]'
SHOWN_FAULTS=$(shown_items 6) || SHOWN_FAULTS='[]'

API_COMPONENTS=$(curl -s -m 30 "${API_BASE}/components")
for id in panda-arm panda-gripper moveit-planning pick-place-loop gateway fault-manager diagnostic-bridge; do
    expected=$(jq -c --arg id "${id}" '.items[] | select(.id == $id) | {id, name, description}' \
        <<< "${API_COMPONENTS}") || expected=""
    shown=$(jq -c --arg id "${id}" '.[] | select(.id == $id) | {id, name, description}' \
        <<< "${SHOWN_COMPONENTS}") || shown=""
    if [ -n "${expected}" ] && [ "${shown}" = "${expected}" ]; then
        pass "check-entities.sh shows component ${id} as the API does"
    else
        fail "check-entities.sh shows component ${id} as the API does" \
            "api=${expected:-none} shown=${shown:-none}"
    fi
done

API_APPS=$(curl -s -m 30 "${API_BASE}/apps")
API_APP_IDS=$(jq -c '[.items[].id] | sort' <<< "${API_APPS}") || API_APP_IDS=""
SHOWN_APP_IDS=$(jq -c '[.[].id] | sort' <<< "${SHOWN_APPS}") || SHOWN_APP_IDS=""
if [ -n "${API_APP_IDS}" ] && [ "${API_APP_IDS}" != "[]" ] && [ "${SHOWN_APP_IDS}" = "${API_APP_IDS}" ]; then
    pass "check-entities.sh lists every app the API lists"
else
    fail "check-entities.sh lists every app the API lists" "api=${API_APP_IDS:-none} shown=${SHOWN_APP_IDS:-none}"
fi
for id in joint-state-broadcaster panda-arm-controller panda-hand-controller robot-state-publisher move-group \
    pick-place-node medkit-gateway medkit-fault-manager diagnostic-bridge-app manipulation-monitor; do
    expected=$(jq -r --arg id "${id}" '.items[] | select(.id == $id) | .["x-medkit"].component_id // empty' \
        <<< "${API_APPS}") || expected=""
    shown=$(jq -r --arg id "${id}" '.[] | select(.id == $id) | .component' <<< "${SHOWN_APPS}") || shown=""
    if [ -n "${expected}" ] && [ "${shown}" = "${expected}" ]; then
        pass "check-entities.sh shows app ${id} on component ${expected}"
    else
        fail "check-entities.sh shows app ${id} on component ${expected:-none}" "shown=${shown:-none}"
    fi
done

FAULT_CODES=$(jq -r '.items[].fault_code' <<< "${API_FAULTS_AFTER}") || FAULT_CODES=""
API_FAULT_COUNT=$(jq '.items | length' <<< "${API_FAULTS_AFTER}") || API_FAULT_COUNT=0
SHOWN_FAULT_COUNT=$(jq 'length' <<< "${SHOWN_FAULTS}") || SHOWN_FAULT_COUNT=0
if [ "${API_FAULT_COUNT:-0}" -gt 0 ] && [ "${SHOWN_FAULT_COUNT:-0}" -eq "${API_FAULT_COUNT}" ]; then
    pass "check-entities.sh lists all ${API_FAULT_COUNT} active faults"
else
    fail "check-entities.sh lists all active faults" "api=${API_FAULT_COUNT} shown=${SHOWN_FAULT_COUNT}"
fi
while IFS= read -r code; do
    [ -n "${code}" ] || continue
    expected=$(jq -c --arg code "${code}" '.items[] | select(.fault_code == $code)
        | [.fault_code, .severity_label, .status, .reporting_sources]' <<< "${API_FAULTS_AFTER}") || expected=""
    shown=$(jq -c --arg code "${code}" '.[] | select(.code == $code) | [.code, .severity, .status, .sources]' \
        <<< "${SHOWN_FAULTS}") || shown=""
    if [ -n "${expected}" ] && [ "${shown}" = "${expected}" ]; then
        pass "check-entities.sh shows fault ${code} with the API's code, severity, status and sources"
    else
        fail "check-entities.sh shows fault ${code} with the API's code, severity, status and sources" \
            "api=${expected:-none} shown=${shown:-none}"
    fi
done <<< "${FAULT_CODES}"

section "move-arm.sh: a local ros2 that cannot reach the demo still moves the arm"

# CI runners (and this dev container) have no ROS 2 wired to the demo's
# graph, but `ros2 node list` still exits 0 on an empty graph. A fake ros2
# that only answers `node list` reproduces that false positive without
# needing a real, disconnected ROS 2 install.
FAKE_ROS2_DIR=$(mktemp -d)
cat > "${FAKE_ROS2_DIR}/ros2" <<'FAKE'
#!/bin/sh
if [ "$1" = "node" ] && [ "$2" = "list" ]; then
    exit 0
fi
exit 1
FAKE
chmod +x "${FAKE_ROS2_DIR}/ros2"

UNREACHABLE_LOG=/tmp/moveit_smoke_unreachable_ros2.log
stop_pick_place_loop
PATH="${FAKE_ROS2_DIR}:${PATH}" run_move_arm "${UNREACHABLE_LOG}" extended
resume_pick_place_loop
rm -rf "${FAKE_ROS2_DIR}"

if [ "${MOVE_ARM_RC}" -eq 0 ] && [ "$(real_status "${UNREACHABLE_LOG}")" = "SUCCEEDED" ]; then
    pass "move-arm.sh moves the container's arm despite a local unreachable ros2"
else
    fail "move-arm.sh moves the container's arm despite a local unreachable ros2" \
        "rc=${MOVE_ARM_RC}; tail: $(tail -n 5 "${UNREACHABLE_LOG}")"
fi

if grep -q 'cannot attach stdin' "${UNREACHABLE_LOG}"; then
    fail "move-arm.sh works without a TTY" "docker exec still requires a TTY"
else
    pass "move-arm.sh works without a TTY"
fi

section "move-arm.sh: a goal the controller never answered is sent again"

# The controller can fail to deliver a goal response to a new ros2 CLI whose
# endpoints are not yet discovered ("Failed to send goal response"). It then
# never runs the goal, and the CLI waits forever. This fake ros2 loses the
# first goal that way and hands the next one to the real CLI in the container.
FAKE_ROS2_DIR=$(mktemp -d)
cat > "${FAKE_ROS2_DIR}/ros2" <<'FAKE'
#!/bin/sh
if [ "$1 $2" = "action list" ]; then
    echo /panda_arm_controller/follow_joint_trajectory
    exit 0
fi
[ "$1 $2" = "action send_goal" ] || exit 1
shift 2
calls=$(($(cat "${FAKE_ROS2_DIR}/calls" 2> /dev/null || echo 0) + 1))
echo "${calls}" > "${FAKE_ROS2_DIR}/calls"
if [ "${calls}" -eq 1 ]; then
    echo "Waiting for an action server to become available..."
    echo "Sending goal:"
    echo "$$" > "${FAKE_ROS2_DIR}/hung.pid"
    exec sleep 3600
fi
exec docker exec -i "${CONTAINER_NAME}" bash -s -- "$@" <<'REMOTE'
set +u
source /opt/ros/jazzy/setup.bash
source /root/demo_ws/install/setup.bash
exec timeout 60 ros2 action send_goal "$@"
REMOTE
FAKE
chmod +x "${FAKE_ROS2_DIR}/ros2"

LOST_LOG=/tmp/moveit_smoke_goal_lost.log
stop_pick_place_loop
export FAKE_ROS2_DIR
if PATH="${FAKE_ROS2_DIR}:${PATH}" CONTAINER_NAME="${DEMO_CONTAINER}" \
    timeout 120 "${DEMO_DIR}/move-arm.sh" wave < /dev/null > "${LOST_LOG}" 2>&1; then
    LOST_RC=0
else
    LOST_RC=$?
fi
resume_pick_place_loop
if [ -f "${FAKE_ROS2_DIR}/hung.pid" ]; then
    kill "$(cat "${FAKE_ROS2_DIR}/hung.pid")" 2> /dev/null || true
fi
LOST_CALLS=$(cat "${FAKE_ROS2_DIR}/calls" 2> /dev/null || echo 0)
rm -rf "${FAKE_ROS2_DIR}"
unset FAKE_ROS2_DIR

if [ "${LOST_RC}" -ne 124 ] && [ "${LOST_CALLS}" -eq 2 ]; then
    pass "move-arm.sh sends the goal again when no goal response arrives"
else
    fail "move-arm.sh sends the goal again when no goal response arrives" \
        "rc=${LOST_RC} (124: still waiting after 120 s), goals sent: ${LOST_CALLS}"
fi
if [ "$(real_status "${LOST_LOG}")" = "SUCCEEDED" ]; then
    pass "setup: the goal sent again really succeeded"
else
    fail "setup: the goal sent again really succeeded" "real status: $(real_status "${LOST_LOG}")"
fi
check_goal_report "${LOST_LOG}" "${LOST_RC}" "wave"

section "move-arm.sh: reports the goal's real final status"

PREEMPTOR_LOG=/tmp/moveit_smoke_preemptor.log
PREEMPTED_LOG=/tmp/moveit_smoke_goal_preempted.log
FREE_LOG=/tmp/moveit_smoke_goal_free.log

# The preempt probe aborts the first goal move-arm.sh starts. The second goal
# runs on an idle arm.
stop_pick_place_loop
start_preemptor "${PREEMPTOR_LOG}" || fail "setup: competing goal ready" "$(tail -n 3 "${PREEMPTOR_LOG}")"
run_move_arm "${PREEMPTED_LOG}" ready
PREEMPTED_RC=${MOVE_ARM_RC}
finish_preemptor "${PREEMPTOR_LOG}"
run_move_arm "${FREE_LOG}" place
FREE_RC=${MOVE_ARM_RC}
resume_pick_place_loop

PREEMPTED_STATUS=$(real_status "${PREEMPTED_LOG}")
if [ -n "${PREEMPTED_STATUS}" ] && [ "${PREEMPTED_STATUS}" = "${PREEMPTED_GOAL_STATUS}" ]; then
    pass "setup: move-arm.sh's goal is the one the probe preempted (${PREEMPTED_STATUS})"
else
    fail "setup: move-arm.sh's goal is the one the probe preempted" \
        "move-arm.sh real status: ${PREEMPTED_STATUS:-none}; preempted goal: ${PREEMPTED_GOAL_STATUS:-none}"
fi
if [ "$(real_status "${FREE_LOG}")" = "SUCCEEDED" ]; then
    pass "setup: a goal on an idle arm really succeeded"
else
    fail "setup: a goal on an idle arm really succeeded" "real status: $(real_status "${FREE_LOG}")"
fi

check_goal_report "${PREEMPTED_LOG}" "${PREEMPTED_RC}" "ready"
check_goal_report "${FREE_LOG}" "${FREE_RC}" "place"

section "move-arm.sh demo: per-step reporting and overall exit code"

DEMO_LOG=/tmp/moveit_smoke_demo.log

# The preempt probe aborts the cycle's first step. The later steps run on an
# idle arm.
stop_pick_place_loop
start_preemptor "${PREEMPTOR_LOG}" || fail "setup: competing goal ready" "$(tail -n 3 "${PREEMPTOR_LOG}")"
run_move_arm "${DEMO_LOG}" demo
DEMO_RC=${MOVE_ARM_RC}
finish_preemptor "${PREEMPTOR_LOG}"
resume_pick_place_loop

DEMO_STEPS=$(grep -c '^🤖 Moving to:' "${DEMO_LOG}" || true)
if [ "${DEMO_STEPS}" -eq 3 ]; then
    pass "move-arm.sh demo runs three steps"
else
    fail "move-arm.sh demo runs three steps" "steps started: ${DEMO_STEPS}"
fi

# Each step block runs from its "Moving to" line to the next one. Its action
# must reach a final status, its result line must match that status, and each
# label must have exactly one result line in the whole output.
DEMO_FAILED_STEPS=0
DEMO_FIRST_STATUS=""
step=0
for label in "pick" "place" "ready (home)"; do
    step=$((step + 1))
    step_log="/tmp/moveit_smoke_demo_step${step}.log"
    awk -v n="${step}" 'index($0, "🤖 Moving to:") == 1 { count++ } count == n' "${DEMO_LOG}" > "${step_log}"
    status=$(real_status "${step_log}")
    [ "${step}" -eq 1 ] && DEMO_FIRST_STATUS="${status}"
    if [ -z "${status}" ]; then
        fail "move-arm.sh demo step ${step} (${label}): the action reports a final status" \
            "no 'Goal finished with status' line; step tail: $(tail -n 3 "${step_log}" | tr '\n' ';')"
        continue
    fi
    if [ "${status}" != "SUCCEEDED" ]; then
        DEMO_FAILED_STEPS=$((DEMO_FAILED_STEPS + 1))
    fi
    first_line=$(head -n 1 "${step_log}")
    label_results=$(( $(grep -cFx "✅ Done: ${label}" "${DEMO_LOG}" || true) \
        + $(grep -cF "Failed: ${label} (" "${DEMO_LOG}" || true) ))
    if [ "${first_line}" = "🤖 Moving to: ${label}" ] && [ "${label_results}" -eq 1 ] \
        && reports_status "${step_log}" "${label}" "${status}"; then
        pass "move-arm.sh demo step ${step} (${label}): one result line, matching real ${status}"
    else
        fail "move-arm.sh demo step ${step} (${label}): one result line, matching real ${status}" \
            "first line: ${first_line}; results for label: ${label_results}; in step: $(grep -E '^(✅ Done|Failed):' "${step_log}" | tr '\n' ';')"
    fi
done

if [ -n "${DEMO_FIRST_STATUS}" ] && [ "${DEMO_FIRST_STATUS}" = "${PREEMPTED_GOAL_STATUS}" ]; then
    pass "setup: the demo's pick step is the goal the probe preempted (${DEMO_FIRST_STATUS})"
else
    fail "setup: the demo's pick step is the goal the probe preempted" \
        "pick real status: ${DEMO_FIRST_STATUS:-none}; preempted goal: ${PREEMPTED_GOAL_STATUS:-none}"
fi
if [ "${DEMO_FAILED_STEPS}" -gt 0 ]; then
    if [ "${DEMO_RC}" -ne 0 ]; then
        pass "move-arm.sh demo exits non-zero when a step really failed"
    else
        fail "move-arm.sh demo exits non-zero when a step really failed" "rc=0"
    fi
else
    fail "setup: a demo step really failed" "no step reached a non-SUCCEEDED final status"
fi

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
