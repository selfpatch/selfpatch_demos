#!/bin/bash
# Move the Panda arm to preset positions via ros2_control action interface.
# Works with fake hardware (mock controllers) — no MoveIt planning needed.
#
# Usage:
#   ./move-arm.sh                  # Interactive menu
#   ./move-arm.sh ready            # Go to ready pose
#   ./move-arm.sh extended         # Extend arm forward
#   ./move-arm.sh pick             # Go to pick pose
#   ./move-arm.sh place            # Go to place pose
#   ./move-arm.sh home             # All joints to zero

set -eu

ACTION="/panda_arm_controller/follow_joint_trajectory"
JOINT_NAMES='["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]'

# Duration in seconds for trajectory execution
DURATION_SEC=3

# Time limit for one `ros2 action send_goal` run, and how many runs per goal.
# The controller can fail to deliver the goal response to a new CLI
# ("Failed to send goal response"). It then never runs the goal and the CLI
# waits forever, so a goal with no response is sent again.
SEND_TIMEOUT_SEC=30
SEND_ATTEMPTS=3

# --- Preset joint positions (radians) ---
# Ready: default MoveIt pose (from SRDF)
READY="[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"

# Home: all joints at zero
HOME="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"

# Extended: arm stretched forward
EXTENDED="[0.0, -0.3, 0.0, -1.5, 0.0, 1.2, 0.785]"

# Pick: reaching down to pick position
PICK="[0.0, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785]"

# Place: rotated to place position
PLACE="[1.2, -0.5, 0.0, -2.0, 0.0, 1.5, 0.785]"

# Left: arm rotated left
LEFT="[-1.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"

# Right: arm rotated right
RIGHT="[1.5, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]"

# Wave: arm raised for waving
WAVE="[0.0, -1.0, 0.0, -0.5, 0.0, 2.5, 0.785]"


have() {
    command -v "$1" &> /dev/null
}

# True only if a LOCAL ros2 can actually reach the target action server.
# `ros2 node list` exits 0 even on an empty graph (wrong ROS_DOMAIN_ID, no
# multicast route), so a host with ROS 2 sourced but not connected to the
# demo looks identical to being inside the container. Checking that the
# action itself is listed avoids that false positive. A cold listing (no
# ros2 daemon yet) can miss a running server, so a miss is listed once more.
can_reach_action_locally() {
    have ros2 || return 1
    ros2 action list 2> /dev/null | grep -qFx "${ACTION}" \
        || ros2 action list 2> /dev/null | grep -qFx "${ACTION}"
}

# Picks how goals are sent: USE_LOCAL=true for the local ros2, else
# `docker exec` into CONTAINER. Without a docker CLI the script runs inside
# the container or on a ROS host, so the local ros2 is the only way.
# On failure sets NOT_SENT_REASON and returns 1.
USE_LOCAL=""
CONTAINER=""
NOT_SENT_REASON=""
choose_transport() {
    [[ -z "${USE_LOCAL}" ]] || return 0
    if ! have docker; then
        if ! have ros2; then
            NOT_SENT_REASON="needs the docker CLI or a sourced ROS 2 (ros2 CLI), found neither"
            return 1
        fi
        USE_LOCAL=true
        return 0
    fi
    if can_reach_action_locally; then
        USE_LOCAL=true
        return 0
    fi
    CONTAINER="${CONTAINER_NAME:-$(docker ps --format '{{.Names}}' | grep -E '^moveit_medkit_demo(_nvidia)?(_local)?$' | head -n1)}"
    if [[ -z "${CONTAINER}" ]]; then
        NOT_SENT_REASON="no running moveit_medkit_demo container, start it with ./run-demo.sh or set CONTAINER_NAME"
        return 1
    fi
    USE_LOCAL=false
}

send_trajectory() {
    local positions="$1"
    local label="$2"

    echo "🤖 Moving to: ${label}"
    echo "   Joints: ${positions}"
    echo ""

    # Build FollowJointTrajectory goal message
    local goal_msg="{
        trajectory: {
            joint_names: ${JOINT_NAMES},
            points: [{
                positions: ${positions},
                time_from_start: {sec: ${DURATION_SEC}, nanosec: 0}
            }]
        }
    }"

    if ! choose_transport; then
        echo "Failed: ${label} (goal not sent: ${NOT_SENT_REASON})" >&2
        return 1
    fi

    # `ros2 action send_goal` always exits 0, whatever the goal's outcome -
    # the real result is in its own printed "Goal finished with status:"
    # line, so capture output and parse that instead of the exit code.
    # PYTHONUNBUFFERED keeps the lines printed before a timeout kills the CLI.
    local attempt output rc
    for ((attempt = 1; attempt <= SEND_ATTEMPTS; attempt++)); do
        rc=0
        if [[ "${USE_LOCAL}" == true ]]; then
            output=$(PYTHONUNBUFFERED=1 timeout "${SEND_TIMEOUT_SEC}" \
                ros2 action send_goal "${ACTION}" \
                control_msgs/action/FollowJointTrajectory \
                "${goal_msg}" \
                --feedback 2>&1) || rc=$?
        else
            # Outside the container: exec into it. No -it: this must also
            # work without a TTY (CI, a pipe), and the command needs no stdin.
            # timeout runs in the container: killing `docker exec` would
            # leave the CLI running there.
            output=$(docker exec "${CONTAINER}" bash -c "
                source /opt/ros/jazzy/setup.bash && \
                source /root/demo_ws/install/setup.bash && \
                PYTHONUNBUFFERED=1 timeout ${SEND_TIMEOUT_SEC} \
                ros2 action send_goal ${ACTION} \
                    control_msgs/action/FollowJointTrajectory \
                    \"${goal_msg}\" \
                    --feedback
            " 2>&1) || rc=$?
        fi
        printf '%s\n' "${output}"
        # An accepted or rejected goal has its answer.
        if grep -qE '^(Goal accepted with ID|Goal was rejected)' <<< "${output}"; then
            break
        fi
        # Never sent (no container, no action server, a CLI error): sending
        # again changes nothing. rc 124 is the timeout.
        if ! grep -q '^Sending goal:' <<< "${output}"; then
            if ((rc == 124)) && grep -q '^Waiting for an action server' <<< "${output}"; then
                echo "Failed: ${label} (goal not sent: no action server ${ACTION} within ${SEND_TIMEOUT_SEC} s)" >&2
            elif ((rc == 124)); then
                echo "Failed: ${label} (goal not sent: timed out after ${SEND_TIMEOUT_SEC} s)" >&2
            else
                echo "Failed: ${label} (goal not sent: exit status ${rc})" >&2
            fi
            return 1
        fi
        # Sent with no response: the controller never runs it, so send again.
        if ((attempt < SEND_ATTEMPTS)); then
            if ((rc == 124)); then
                echo "No goal response within ${SEND_TIMEOUT_SEC} s, sending the goal again"
            else
                echo "No goal response (exit status ${rc}), sending the goal again"
            fi
        fi
    done

    local status
    status=$(printf '%s\n' "${output}" | grep -F 'Goal finished with status:' | tail -n1 | sed -E 's/.*status: *//')

    echo ""
    if [[ "${status}" == "SUCCEEDED" ]]; then
        echo "✅ Done: ${label}"
        return 0
    fi
    echo "Failed: ${label} (status: ${status:-UNKNOWN})" >&2
    return 1
}

show_menu() {
    echo ""
    echo "🤖 Panda Arm Controller"
    echo "========================"
    echo ""
    echo "Preset positions:"
    echo "  1) ready     — Default MoveIt pose (relaxed)"
    echo "  2) home      — All joints at zero"
    echo "  3) extended  — Arm stretched forward"
    echo "  4) pick      — Reaching down to pick"
    echo "  5) place     — Rotated to place position"
    echo "  6) left      — Arm rotated left"
    echo "  7) right     — Arm rotated right"
    echo "  8) wave      — Arm raised high"
    echo ""
    echo "  d) demo      — Run full pick-place-home cycle"
    echo "  q) quit"
    echo ""
}

run_demo_cycle() {
    echo "🔄 Running pick → place → home cycle..."
    echo ""
    local failed=0
    send_trajectory "${PICK}" "pick" || failed=1
    sleep 2
    send_trajectory "${PLACE}" "place" || failed=1
    sleep 2
    send_trajectory "${READY}" "ready (home)" || failed=1
    echo ""
    if [[ "${failed}" -eq 0 ]]; then
        echo "🔄 Cycle complete!"
    else
        echo "🔄 Cycle complete with failures" >&2
    fi
    return "${failed}"
}

handle_choice() {
    local choice="$1"
    case "$choice" in
        1|ready)     send_trajectory "${READY}" "ready" ;;
        2|home)      send_trajectory "${HOME}" "home" ;;
        3|extended)  send_trajectory "${EXTENDED}" "extended" ;;
        4|pick)      send_trajectory "${PICK}" "pick" ;;
        5|place)     send_trajectory "${PLACE}" "place" ;;
        6|left)      send_trajectory "${LEFT}" "left" ;;
        7|right)     send_trajectory "${RIGHT}" "right" ;;
        8|wave)      send_trajectory "${WAVE}" "wave" ;;
        d|demo)      run_demo_cycle ;;
        q|quit|exit) echo "Bye!"; exit 0 ;;
        *)           echo "Unknown option: ${choice}" ;;
    esac
}

# --- Main ---

# If argument provided, run directly. Reflect the goal's real result in the
# exit code instead of always exiting 0.
if [[ $# -gt 0 ]]; then
    if handle_choice "$1"; then
        exit 0
    else
        exit 1
    fi
fi

# Interactive mode. A failed goal reports failure and the menu continues -
# it must not kill the session (set -e would, without this guard).
while true; do
    show_menu
    read -rp "Choose position (1-8, d, q): " choice
    handle_choice "${choice}" || true
done
