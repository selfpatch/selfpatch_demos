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

# Add a large wall via /apply_planning_scene service (reliable)
echo "Adding collision wall to planning scene..."
docker exec moveit_medkit_demo bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
python3 -c \"
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header

rclpy.init()
node = Node('inject_wall')
client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
client.wait_for_service(timeout_sec=5.0)

scene = PlanningScene()
scene.is_diff = True

wall = CollisionObject()
wall.id = 'injected_wall'
wall.header.frame_id = 'panda_link0'
wall.operation = CollisionObject.ADD
prim = SolidPrimitive()
prim.type = SolidPrimitive.BOX
prim.dimensions = [2.0, 0.05, 1.0]
wall.primitives.append(prim)
pose = Pose()
pose.position.x = 0.3
pose.position.y = 0.25
pose.position.z = 0.5
pose.orientation.w = 1.0
wall.primitive_poses.append(pose)
scene.world.collision_objects.append(wall)

req = ApplyPlanningScene.Request()
req.scene = scene
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
result = future.result()
print(f'Wall added: {result.success}' if result else 'Service call failed')
node.destroy_node()
rclpy.shutdown()
\""

echo ""
echo "✓ Planning failure injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: MoveGroup goal ABORTED"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
