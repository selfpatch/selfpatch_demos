#!/bin/bash
# Restore Normal Operation - Remove all injected faults
# Runs INSIDE the Docker container.

set -e

source /opt/ros/jazzy/setup.bash
source /root/demo_ws/install/setup.bash

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "🔄 Restoring NORMAL operation..."
echo ""

# 1. Remove injected Gazebo models (visual objects)
echo "Removing injected Gazebo models..."
for MODEL_NAME in surprise_obstacle injected_wall; do
  if gz service -s /world/panda_factory/remove \
      --reqtype gz.msgs.Entity \
      --reptype gz.msgs.Boolean \
      --timeout 3000 \
      --req "name: \"${MODEL_NAME}\" type: MODEL" 2>/dev/null; then
    echo "  ${MODEL_NAME}: removed"
  else
    echo "  ${MODEL_NAME}: not found (ok)"
  fi
done

# 2. Remove injected collision objects from MoveIt planning scene
echo "Removing injected collision objects from planning scene..."
python3 -c "
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject

rclpy.init()
node = Node('restore_helper')

client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
client.wait_for_service(timeout_sec=5.0)

scene = PlanningScene()
scene.is_diff = True

for obj_id in ['injected_wall', 'surprise_obstacle']:
    obj = CollisionObject()
    obj.id = obj_id
    obj.operation = CollisionObject.REMOVE
    scene.world.collision_objects.append(obj)

req = ApplyPlanningScene.Request()
req.scene = scene
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
result = future.result()
print(f'Planning scene cleaned: {result.success}' if result else 'Service call failed')
node.destroy_node()
rclpy.shutdown()
"

# 3. Wait for operations to stabilize, then clear faults
echo "Waiting for operations to stabilize..."
sleep 10
echo "Clearing all faults..."
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true
# Wait for any straggling reports to land, then clear again
sleep 5
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true

echo ""
echo "✓ Normal operation restored!"
echo ""
if command -v jq >/dev/null 2>&1; then
  echo "Current fault status:"
  curl -sf "${API_BASE}/faults" 2>/dev/null | jq '.items | length' | xargs -I {} echo "  Active faults: {}" || echo "  (could not query faults)"
fi
echo ""
echo "Robot is ready for normal operation."
