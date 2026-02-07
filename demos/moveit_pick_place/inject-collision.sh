#!/bin/bash
# Inject Collision - Spawn a surprise obstacle in the robot's workspace
# This will cause planning/execution failures due to unexpected collision object

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🚫 Injecting COLLISION fault..."
echo "   Spawning surprise obstacle in robot workspace"
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

# Add a sphere right in the arm's working area
echo "Adding surprise obstacle (sphere) to planning scene..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 topic pub --once /planning_scene moveit_msgs/msg/PlanningScene '{
  world: {
    collision_objects: [
      {
        id: \"surprise_obstacle\",
        header: {frame_id: \"panda_link0\"},
        primitives: [{type: 2, dimensions: [0.15]}],
        primitive_poses: [{
          position: {x: 0.4, y: 0.0, z: 0.4},
          orientation: {w: 1.0}
        }],
        operation: 0
      }
    ]
  },
  is_diff: true
}'"

echo ""
echo "✓ Collision fault injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: Cannot find collision-free path"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
