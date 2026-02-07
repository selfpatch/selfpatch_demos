#!/bin/bash
# Inject Joint Limit Violation - Command extreme joint positions
# This will trigger joint limit approaching/violation faults

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting JOINT LIMIT fault..."
echo "   Commanding extreme joint positions near limits"
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

# Send a goal with extreme joint positions via ROS 2 CLI
echo "Sending joint trajectory goal near limits..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory '{
    trajectory: {
      joint_names: [panda_joint1, panda_joint2, panda_joint3,
                    panda_joint4, panda_joint5, panda_joint6, panda_joint7],
      points: [{
        positions: [2.85, 1.70, 2.85, -0.10, 2.85, 3.70, 2.85],
        time_from_start: {sec: 5, nanosec: 0}
      }]
    }
  }'" 2>/dev/null &

echo ""
echo "✓ Joint limit fault injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - JOINT_LIMIT_APPROACHING: Joint near limit (WARN)"
echo "  - JOINT_LIMIT_VIOLATED: Joint beyond limit (ERROR)"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
