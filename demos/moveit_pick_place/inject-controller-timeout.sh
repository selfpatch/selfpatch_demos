#!/bin/bash
# Inject Controller Timeout - Set extremely low velocity scaling
# This will cause the arm controller to timeout during trajectory execution

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting CONTROLLER TIMEOUT fault..."
echo "   Setting extremely tight goal time tolerance (0.001s)"
echo "   This forces the controller to abort — it can't reach the target in time"
echo ""

# Check for jq dependency
if ! command -v jq >/dev/null 2>&1; then
    echo "❌ 'jq' is required but not installed."
    echo "   Please install jq (e.g., 'sudo apt-get install jq') and retry."
    exit 1
fi

# Check gateway
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    exit 1
fi

# Set an extremely tight goal_time constraint on the controller
# This causes FollowJointTrajectory to abort because the robot can't
# reach the target within the allowed time window
echo "Setting controller goal_time constraint via ROS 2 parameter..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 param set /panda_arm_controller constraints.goal_time 0.001" 2>/dev/null || \
    echo "   Warning: Could not set goal_time parameter"

# Also try setting stopped_velocity_tolerance to near-zero to make it
# harder for the controller to consider the trajectory complete
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 param set /panda_arm_controller constraints.stopped_velocity_tolerance 0.0001" 2>/dev/null || \
    echo "   Warning: Could not set stopped_velocity_tolerance parameter"

echo ""
echo "✓ Controller timeout injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - CONTROLLER_TIMEOUT: Joint trajectory controller timed out"
echo "  - TRAJECTORY_EXECUTION_FAILED: Arm controller ABORTED trajectory"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
