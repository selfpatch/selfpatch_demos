#!/bin/bash
# Inject Grasp Failure - Move target object out of robot's workspace
# This will cause MoveGroup to fail planning to an unreachable target

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting GRASP FAILURE fault..."
echo "   Moving target object far outside robot's workspace"
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

# Move the target cylinder far away from the robot
echo "Moving target object to unreachable position (5.0, 5.0, 0.1)..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 topic pub --once /planning_scene moveit_msgs/msg/PlanningScene '{
  world: {
    collision_objects: [
      {
        id: \"target_cylinder\",
        header: {frame_id: \"panda_link0\"},
        primitives: [{type: 3, dimensions: [0.1, 0.02]}],
        primitive_poses: [{
          position: {x: 5.0, y: 5.0, z: 0.1},
          orientation: {w: 1.0}
        }],
        operation: 0
      }
    ]
  },
  is_diff: true
}'"

echo ""
echo "✓ Grasp failure injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: Cannot plan to unreachable target"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
