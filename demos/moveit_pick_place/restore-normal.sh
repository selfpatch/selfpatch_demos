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

# 1. Remove injected collision objects via /apply_planning_scene service
# Using service call (not topic pub) for reliable planning scene updates
echo "Removing injected collision objects..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
python3 -c \"
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from std_msgs.msg import Header

rclpy.init()
node = Node('restore_helper')

# Remove injected objects
client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
client.wait_for_service(timeout_sec=5.0)

scene = PlanningScene()
scene.is_diff = True

# Remove injected_wall
wall = CollisionObject()
wall.id = 'injected_wall'
wall.operation = CollisionObject.REMOVE
scene.world.collision_objects.append(wall)

# Remove surprise_obstacle
obstacle = CollisionObject()
obstacle.id = 'surprise_obstacle'
obstacle.operation = CollisionObject.REMOVE
scene.world.collision_objects.append(obstacle)

# Remove target_cylinder (moved by grasp failure)
cylinder = CollisionObject()
cylinder.id = 'target_cylinder'
cylinder.operation = CollisionObject.REMOVE
scene.world.collision_objects.append(cylinder)

req = ApplyPlanningScene.Request()
req.scene = scene
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
result = future.result()
print(f'Scene update accepted: {result.success}' if result else 'Service call failed')
node.destroy_node()
rclpy.shutdown()
\"
"

# 3. Restore controller parameters via gateway REST API
echo "Restoring controller parameters..."
curl -s -X PUT "${API_BASE}/apps/panda-arm-controller/configurations/constraints.goal_time" \
  -H "Content-Type: application/json" -d '{"data": 0.0}' > /dev/null 2>&1 || true
curl -s -X PUT "${API_BASE}/apps/panda-arm-controller/configurations/constraints.stopped_velocity_tolerance" \
  -H "Content-Type: application/json" -d '{"data": 0.01}' > /dev/null 2>&1 || true

# 4. Wait for operations to stabilize, then clear faults
echo "Waiting for operations to stabilize..."
sleep 10
echo "Clearing all faults..."
curl -s -X DELETE "${API_BASE}/faults" > /dev/null
# Wait for any straggling reports to land, then clear again
sleep 5
curl -s -X DELETE "${API_BASE}/faults" > /dev/null

echo ""
echo "✓ Normal operation restored!"
echo ""
echo "Current fault status:"
curl -s "${API_BASE}/faults" | jq '.items | length' | xargs -I {} echo "  Active faults: {}"
echo ""
echo "Robot is ready for normal operation."
