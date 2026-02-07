#!/bin/bash
# Inject Planning Failure - Add collision wall blocking the pick-place path
# This will cause MoveGroup planning to fail

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting PLANNING FAILURE fault..."
echo "   Adding collision wall between pick and place positions"
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

# Add a large wall via PlanningScene topic inside the container
echo "Adding collision wall to planning scene..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 topic pub --once /planning_scene moveit_msgs/msg/PlanningScene '{
  world: {
    collision_objects: [
      {
        id: \"injected_wall\",
        header: {frame_id: \"panda_link0\"},
        primitives: [{type: 1, dimensions: [2.0, 0.05, 1.0]}],
        primitive_poses: [{
          position: {x: 0.3, y: 0.25, z: 0.5},
          orientation: {w: 1.0}
        }],
        operation: 0
      }
    ]
  },
  is_diff: true
}'"

echo ""
echo "✓ Planning failure injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: MoveGroup goal ABORTED"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
