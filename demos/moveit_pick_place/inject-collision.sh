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

# Add a sphere right in the arm's working area via /apply_planning_scene service
echo "Adding surprise obstacle (sphere) to planning scene..."
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
node = Node('inject_obstacle')
client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
client.wait_for_service(timeout_sec=5.0)

scene = PlanningScene()
scene.is_diff = True

obs = CollisionObject()
obs.id = 'surprise_obstacle'
obs.header.frame_id = 'panda_link0'
obs.operation = CollisionObject.ADD
prim = SolidPrimitive()
prim.type = SolidPrimitive.SPHERE
prim.dimensions = [0.15]
obs.primitives.append(prim)
pose = Pose()
pose.position.x = 0.4
pose.position.y = 0.0
pose.position.z = 0.4
pose.orientation.w = 1.0
obs.primitive_poses.append(pose)
scene.world.collision_objects.append(obs)

req = ApplyPlanningScene.Request()
req.scene = scene
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
result = future.result()
print(f'Obstacle added: {result.success}' if result else 'Service call failed')
node.destroy_node()
rclpy.shutdown()
\""

echo ""
echo "✓ Collision fault injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: Cannot find collision-free path"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
