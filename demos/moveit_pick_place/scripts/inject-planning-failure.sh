#!/bin/bash
# Inject Planning Failure - Add collision wall blocking the pick-place path
# Adds the wall to both Gazebo (visible) and MoveIt planning scene (causes faults)
# Runs INSIDE the Docker container.

set -e

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash

echo "🚫 Injecting PLANNING FAILURE fault..."
echo "   Adding collision wall between pick and place positions"
echo ""

# 1. Spawn visible wall in Gazebo
# Robot base (panda_link0) is at z=0.75 in the world frame.
# Wall at panda_link0 frame (0.3, 0.25, 0.5) → world frame (0.3, 0.25, 1.25)
# Wall dimensions: 2.0 x 0.05 x 1.0 (wide, thin, tall)
echo "Spawning visible orange wall in Gazebo..."
cat > /tmp/injected_wall.sdf << 'EOSDF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="injected_wall">
    <static>true</static>
    <link name="link">
      <visual name="vis">
        <geometry><box><size>2.0 0.05 1.0</size></box></geometry>
        <material>
          <ambient>0.9 0.6 0.1 1</ambient>
          <diffuse>0.95 0.65 0.15 1</diffuse>
          <specular>0.4 0.3 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOSDF

if ros2 run ros_gz_sim create \
    -file /tmp/injected_wall.sdf \
    -name injected_wall \
    -x 0.3 -y 0.25 -z 1.25 2>&1 | tail -1; then
  echo "  ✓ Gazebo model spawned"
else
  echo "  ⚠ Gazebo spawn failed (visual only — fault injection still works)"
fi

# 2. Add to MoveIt planning scene (so planner cannot find a path)
echo "Adding collision wall to MoveIt planning scene..."
python3 -c "
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

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
print(f'Planning scene updated: {result.success}' if result else 'Service call failed')
node.destroy_node()
rclpy.shutdown()
"

echo ""
echo "✓ Planning failure injected!"
echo "  An orange wall is now visible in Gazebo and registered in MoveIt planning scene."
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: MoveGroup goal ABORTED — no collision-free path"
echo ""
echo "Restore with: /root/demo_ws/scripts/restore-normal.sh"
