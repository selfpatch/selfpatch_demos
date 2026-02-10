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

CONTAINER="${CONTAINER_NAME:-$(docker ps --format '{{.Names}}' | grep -E '^moveit_medkit_demo(_nvidia)?(_local)?$' | head -n1)}"
ACTION="/panda_arm_controller/follow_joint_trajectory"
JOINT_NAMES='["panda_joint1","panda_joint2","panda_joint3","panda_joint4","panda_joint5","panda_joint6","panda_joint7"]'

# Duration in seconds for trajectory execution
DURATION_SEC=3

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

    # Check if we're inside the container or outside
    if command -v ros2 &> /dev/null && ros2 node list &> /dev/null 2>&1; then
        # Inside the container (or ROS 2 env is set up)
        ros2 action send_goal "${ACTION}" \
            control_msgs/action/FollowJointTrajectory \
            "${goal_msg}" \
            --feedback
    else
        # Outside — exec into container
        docker exec -it "${CONTAINER}" bash -c "
            source /opt/ros/jazzy/setup.bash && \
            source /root/demo_ws/install/setup.bash && \
            ros2 action send_goal ${ACTION} \
                control_msgs/action/FollowJointTrajectory \
                \"${goal_msg}\" \
                --feedback
        "
    fi

    echo ""
    echo "✅ Done: ${label}"
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
    send_trajectory "${PICK}" "pick"
    sleep 2
    send_trajectory "${PLACE}" "place"
    sleep 2
    send_trajectory "${READY}" "ready (home)"
    echo ""
    echo "🔄 Cycle complete!"
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

# If argument provided, run directly
if [[ $# -gt 0 ]]; then
    handle_choice "$1"
    exit 0
fi

# Interactive mode
while true; do
    show_menu
    read -rp "Choose position (1-8, d, q): " choice
    handle_choice "${choice}"
done
