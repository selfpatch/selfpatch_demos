#!/bin/bash
# Inject Grasp Failure - Move target object out of robot's workspace
# This will cause MoveGroup to fail planning to an unreachable target
#
# NOTE: This injection only works if the pick-place loop uses the
# target_cylinder object as its grasp target. If the loop uses hardcoded
# joint/cartesian targets, moving the collision object has no effect.

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

# Move the target cylinder far away from the robot via /apply_planning_scene service
echo "Moving target object to unreachable position (5.0, 5.0, 0.1)..."

# Auto-detect running demo container (supports CPU and NVIDIA profiles)
CONTAINER="${CONTAINER_NAME:-$(docker ps --format '{{.Names}}' | grep -E '^moveit_medkit_demo(_nvidia)?$' | head -n1)}"
if [ -z "${CONTAINER}" ]; then
    echo "❌ Could not find a running MoveIt demo container."
    echo "   Start the demo first, or set CONTAINER_NAME explicitly."
    exit 1
fi

docker exec "${CONTAINER}" bash -c "
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
node = Node('inject_grasp')
client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
client.wait_for_service(timeout_sec=5.0)

scene = PlanningScene()
scene.is_diff = True

cyl = CollisionObject()
cyl.id = 'target_cylinder'
cyl.header.frame_id = 'panda_link0'
cyl.operation = CollisionObject.ADD
prim = SolidPrimitive()
prim.type = SolidPrimitive.CYLINDER
prim.dimensions = [0.1, 0.02]
cyl.primitives.append(prim)
pose = Pose()
pose.position.x = 5.0
pose.position.y = 5.0
pose.position.z = 0.1
pose.orientation.w = 1.0
cyl.primitive_poses.append(pose)
scene.world.collision_objects.append(cyl)

req = ApplyPlanningScene.Request()
req.scene = scene
future = client.call_async(req)
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
result = future.result()
print(f'Target moved: {result.success}' if result else 'Service call failed')
node.destroy_node()
rclpy.shutdown()
\""

echo ""
echo "✓ Grasp failure injected!"
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: Cannot plan to unreachable target"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
echo "Restore with: ./restore-normal.sh"
