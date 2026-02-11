#!/bin/bash
# Inject Collision - Spawn a surprise obstacle in the robot's workspace
# Adds the object to both Gazebo (visible) and MoveIt planning scene (causes faults)
# Runs INSIDE the Docker container.

set -e

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash

echo "🚫 Injecting COLLISION fault..."
echo "   Spawning surprise obstacle in robot workspace"
echo ""

# 1. Spawn visible model in Gazebo
# Robot base (panda_link0) is at z=0.75 in the world frame.
# Obstacle at panda_link0 frame (0.4, 0, 0.4) → world frame (0.4, 0, 1.15)
echo "Spawning visible red sphere in Gazebo..."
cat > /tmp/surprise_obstacle.sdf << 'EOSDF'
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="surprise_obstacle">
    <static>true</static>
    <link name="link">
      <visual name="vis">
        <geometry><sphere><radius>0.15</radius></sphere></geometry>
        <material>
          <ambient>0.9 0.1 0.1 1</ambient>
          <diffuse>0.95 0.15 0.15 1</diffuse>
          <specular>0.4 0.1 0.1 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOSDF

if ros2 run ros_gz_sim create \
    -file /tmp/surprise_obstacle.sdf \
    -name surprise_obstacle \
    -x 0.4 -y 0.0 -z 1.15 2>&1 | tail -1; then
  echo "  ✓ Gazebo model spawned"
else
  echo "  ⚠ Gazebo spawn failed (visual only — fault injection still works)"
fi

# 2. Add to MoveIt planning scene (so planner detects the collision)
echo "Adding obstacle to MoveIt planning scene..."
python3 -c "
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

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
print(f'Planning scene updated: {result.success}' if result else 'Service call failed')
node.destroy_node()
rclpy.shutdown()
"

echo ""
echo "✓ Collision fault injected!"
echo "  A red sphere is now visible in Gazebo and registered in MoveIt planning scene."
echo ""
echo "Expected faults (via manipulation_monitor → FaultManager):"
echo "  - MOTION_PLANNING_FAILED: Cannot find collision-free path"
echo ""
echo "Restore with: /root/demo_ws/scripts/restore-normal.sh"
