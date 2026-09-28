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

# pick_place_loop sends goals to the same controller action move-arm.sh
# drives directly, so a live loop makes goal outcomes below racy against the
# demo's own workload. Pausing the OS process (no file or behavior change)
# is how the tests below get a deterministic outcome to assert on.
stop_pick_place_loop() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${PICK_PLACE_PATTERN}') || exit 0
        kill -STOP \"\${pid}\"
    " > /dev/null 2>&1 || true
}

resume_pick_place_loop() {
    docker exec "${DEMO_CONTAINER}" bash -c "
        pid=\$(pgrep -f '${PICK_PLACE_PATTERN}') || exit 0
        kill -CONT \"\${pid}\"
    " > /dev/null 2>&1 || true
}

cleanup_on_exit() {
    resume_pick_place_loop
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

section "check-entities.sh: no null fields, including under an active fault"

# Force a real fault: send two goals to the same controller back to back so
# the second preempts the first, a real ABORTED goal that
# manipulation_monitor turns into TRAJECTORY_EXECUTION_FAILED /
# CONTROLLER_TIMEOUT. One rclpy action client (not two `ros2` CLI
# invocations) removes per-invocation DDS discovery jitter, so which goal
# gets preempted is deterministic.
echo "  Forcing a real active fault via controller goal preemption..."
stop_pick_place_loop
docker exec -i "${DEMO_CONTAINER}" bash -s > /tmp/moveit_smoke_fault_setup.log 2>&1 <<'REMOTE' || true
set -eu
set +u
source /opt/ros/jazzy/setup.bash
source /root/demo_ws/install/setup.bash
set -u
python3 - <<'PYEOF'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

JOINTS = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7",
]


def make_goal(positions, sec):
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start = Duration(sec=sec, nanosec=0)
    goal.trajectory.points = [point]
    return goal


rclpy.init()
node = Node("smoke_fault_probe")
client = ActionClient(
    node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory"
)
client.wait_for_server()

ready = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
future_a = client.send_goal_async(make_goal(ready, 3))
rclpy.spin_until_future_complete(node, future_a)
handle_a = future_a.result()

future_b = client.send_goal_async(make_goal(ready, 1))
rclpy.spin_until_future_complete(node, future_b)
handle_b = future_b.result()

rclpy.spin_until_future_complete(node, handle_a.get_result_async())
rclpy.spin_until_future_complete(node, handle_b.get_result_async())

node.destroy_node()
rclpy.shutdown()
PYEOF
REMOTE
resume_pick_place_loop

echo "  Waiting for the fault to appear (max 20s)..."
if poll_until "/faults" '.items | length > 0' 20; then
    pass "setup: a real active fault exists"
else
    fail "setup: a real active fault exists" "no fault after forced controller preemption"
fi

ENTITIES_OUTPUT=$(GATEWAY_URL="${GATEWAY_URL}" "${DEMO_DIR}/check-entities.sh" 2>&1) || true
if ! printf '%s\n' "${ENTITIES_OUTPUT}" | grep -q 'exploration complete'; then
    fail "check-entities.sh runs to completion" \
        "$(printf '%s\n' "${ENTITIES_OUTPUT}" | tail -5)"
elif printf '%s\n' "${ENTITIES_OUTPUT}" | grep -q '": null'; then
    fail "check-entities.sh prints no null fields" \
        "$(printf '%s\n' "${ENTITIES_OUTPUT}" | grep '": null' | sort -u | tr '\n' ';')"
else
    pass "check-entities.sh prints no null fields"
fi

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

stop_pick_place_loop
if MOVE_ARM_OUTPUT=$(PATH="${FAKE_ROS2_DIR}:${PATH}" CONTAINER_NAME="${DEMO_CONTAINER}" \
    "${DEMO_DIR}/move-arm.sh" extended < /dev/null 2>&1); then
    MOVE_ARM_RC=0
else
    MOVE_ARM_RC=$?
fi
resume_pick_place_loop
rm -rf "${FAKE_ROS2_DIR}"

if [ "${MOVE_ARM_RC}" -eq 0 ] && printf '%s\n' "${MOVE_ARM_OUTPUT}" | grep -q 'Goal finished with status: SUCCEEDED'; then
    pass "move-arm.sh moves the container's arm despite a local unreachable ros2"
else
    fail "move-arm.sh moves the container's arm despite a local unreachable ros2" \
        "rc=${MOVE_ARM_RC}; tail: $(printf '%s\n' "${MOVE_ARM_OUTPUT}" | tail -5)"
fi

if printf '%s\n' "${MOVE_ARM_OUTPUT}" | grep -q 'cannot attach stdin'; then
    fail "move-arm.sh works without a TTY" "docker exec still requires a TTY"
else
    pass "move-arm.sh works without a TTY"
fi

section "move-arm.sh: reports the goal's real final status"

# Launch two goals for the same joints at (as close as the shell gets to)
# the same instant. The controller always aborts whichever one it already
# had in flight when the second arrives, so exactly one of the two
# invocations below is guaranteed to see a real ABORTED result and the
# other a real SUCCEEDED one - which one is not predictable, so both are
# checked post hoc.
stop_pick_place_loop
# set +e inside each subshell: it inherits the outer `set -e`, and without
# this a non-zero move-arm.sh exit (exactly what these subshells expect to
# sometimes see) would abort the subshell before it writes its .rc file.
(
    set +e
    CONTAINER_NAME="${DEMO_CONTAINER}" "${DEMO_DIR}/move-arm.sh" ready < /dev/null \
        > /tmp/moveit_smoke_goal_a.log 2>&1
    echo "$?" > /tmp/moveit_smoke_goal_a.rc
) &
GOAL_A_PID=$!
(
    set +e
    CONTAINER_NAME="${DEMO_CONTAINER}" "${DEMO_DIR}/move-arm.sh" place < /dev/null \
        > /tmp/moveit_smoke_goal_b.log 2>&1
    echo "$?" > /tmp/moveit_smoke_goal_b.rc
) &
GOAL_B_PID=$!
wait "${GOAL_A_PID}" "${GOAL_B_PID}"
resume_pick_place_loop

GOAL_A_RC=$(cat /tmp/moveit_smoke_goal_a.rc)
GOAL_B_RC=$(cat /tmp/moveit_smoke_goal_b.rc)
GOAL_A_OUT=$(cat /tmp/moveit_smoke_goal_a.log)
GOAL_B_OUT=$(cat /tmp/moveit_smoke_goal_b.log)

if { [ "${GOAL_A_RC}" -eq 0 ] && [ "${GOAL_B_RC}" -ne 0 ]; } || \
   { [ "${GOAL_A_RC}" -ne 0 ] && [ "${GOAL_B_RC}" -eq 0 ]; }; then
    pass "concurrent goals: one real ABORTED and one real SUCCEEDED occurred"
else
    fail "concurrent goals: one real ABORTED and one real SUCCEEDED occurred" \
        "goal A rc=${GOAL_A_RC}, goal B rc=${GOAL_B_RC}"
fi

# check_goal_report <rc> <output> <label>: a SUCCEEDED goal must exit 0 with
# a success line and no failure line; anything else must exit non-zero with
# a failure line and no success line.
check_goal_report() {
    local rc="$1" out="$2" label="$3"
    local done_lines failed_lines
    done_lines=$(printf '%s\n' "${out}" | grep -c '✅ Done:' || true)
    failed_lines=$(printf '%s\n' "${out}" | grep -c '^Failed:' || true)

    if [ "${rc}" -eq 0 ]; then
        if [ "${done_lines}" -ge 1 ] && [ "${failed_lines}" -eq 0 ]; then
            pass "${label}: SUCCEEDED goal exits 0 with a success line"
        else
            fail "${label}: SUCCEEDED goal exits 0 with a success line" \
                "rc=0 done_lines=${done_lines} failed_lines=${failed_lines}"
        fi
    else
        if [ "${failed_lines}" -ge 1 ] && [ "${done_lines}" -eq 0 ]; then
            pass "${label}: non-SUCCEEDED goal exits non-zero with no success line"
        else
            fail "${label}: non-SUCCEEDED goal exits non-zero with no success line" \
                "rc=${rc} done_lines=${done_lines} failed_lines=${failed_lines}"
        fi
    fi
}

check_goal_report "${GOAL_A_RC}" "${GOAL_A_OUT}" "goal A (ready)"
check_goal_report "${GOAL_B_RC}" "${GOAL_B_OUT}" "goal B (place)"

section "move-arm.sh demo: per-step reporting and overall exit code"

# A single competing goal, fired shortly after the cycle's first ("pick")
# step starts, deterministically preempts that step (see check_goal_report
# section above for why a fixed small delay - not synchronization on
# output - is enough here). Bounded retries absorb the residual scheduling
# jitter of two independent processes without masking a real failure: the
# assertion below still requires an actual failed step every time it passes.
DEMO_OK=0
DEMO_RC=1
DEMO_DONE=0
DEMO_FAILED=0
for attempt in 1 2 3; do
    echo "  Attempt ${attempt}/3..."
    stop_pick_place_loop
    (
        set +e
        CONTAINER_NAME="${DEMO_CONTAINER}" "${DEMO_DIR}/move-arm.sh" demo < /dev/null \
            > /tmp/moveit_smoke_demo.log 2>&1
        echo "$?" > /tmp/moveit_smoke_demo.rc
    ) &
    DEMO_PID=$!
    sleep 0.8
    docker exec "${DEMO_CONTAINER}" bash -c "
        set +u; source /opt/ros/jazzy/setup.bash; source /root/demo_ws/install/setup.bash; set -u
        ros2 action send_goal ${ACTION} control_msgs/action/FollowJointTrajectory \
            \"{trajectory: {joint_names: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], points: [{positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], time_from_start: {sec: 1, nanosec: 0}}]}}\" \
            --feedback
    " > /tmp/moveit_smoke_demo_competitor.log 2>&1 || true
    wait "${DEMO_PID}"
    resume_pick_place_loop

    DEMO_RC=$(cat /tmp/moveit_smoke_demo.rc)
    DEMO_OUT=$(cat /tmp/moveit_smoke_demo.log)
    DEMO_DONE=$(printf '%s\n' "${DEMO_OUT}" | grep -c '✅ Done:' || true)
    DEMO_FAILED=$(printf '%s\n' "${DEMO_OUT}" | grep -c '^Failed:' || true)

    if [ "${DEMO_RC}" -ne 0 ] && [ "${DEMO_FAILED}" -ge 1 ] && [ "${DEMO_DONE}" -ge 1 ]; then
        DEMO_OK=1
        break
    fi
    sleep 2
done

if [ "${DEMO_OK}" -eq 1 ]; then
    pass "move-arm.sh demo: exits non-zero when a step fails"
    pass "move-arm.sh demo: later steps still run and report after an earlier step fails"
else
    fail "move-arm.sh demo: exits non-zero when a step fails" \
        "rc=${DEMO_RC} done_lines=${DEMO_DONE} failed_lines=${DEMO_FAILED} after 3 attempts"
    fail "move-arm.sh demo: later steps still run and report after an earlier step fails" \
        "no run in 3 attempts produced both a failed and a completed step"
fi

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
