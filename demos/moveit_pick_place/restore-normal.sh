#!/bin/bash
# Restore Normal Operation - Reset all injected faults
# Use this after running any inject-*.sh script

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🔄 Restoring NORMAL operation..."
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

# 1. Remove injected collision objects
echo "Removing injected collision objects..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 topic pub --once /planning_scene moveit_msgs/msg/PlanningScene '{
  world: {
    collision_objects: [
      {id: \"injected_wall\", operation: 1},
      {id: \"surprise_obstacle\", operation: 1}
    ]
  },
  is_diff: true
}'"

# 2. Restore target object to original position
echo "Restoring target object to pick position..."
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
          position: {x: 0.5, y: 0.0, z: 0.1},
          orientation: {w: 1.0}
        }],
        operation: 0
      }
    ]
  },
  is_diff: true
}'"

# 3. Restore controller parameters
echo "Restoring controller parameters..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 param set /panda_arm_controller constraints.goal_time 0.0 && \
ros2 param set /panda_arm_controller constraints.stopped_velocity_tolerance 0.01" 2>/dev/null || true

# 4. Clear all faults
echo "Clearing all faults..."
curl -s -X DELETE "${API_BASE}/faults" > /dev/null

echo ""
echo "✓ Normal operation restored!"
echo ""
echo "Current fault status:"
curl -s "${API_BASE}/faults" | jq '.items | length' | xargs -I {} echo "  Active faults: {}"
echo ""
echo "Robot is ready for normal operation."
