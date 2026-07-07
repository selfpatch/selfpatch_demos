#!/bin/bash
# Send a Nav2 goal into the demo container (ROS_DOMAIN_ID=42) via the
# NavigateToPose action. Starts the mission (robot drives into the phantom
# sector) and resumes it after the fix.
#
# This goes through the /navigate_to_pose action instead of a one-shot
# `ros2 topic pub --once /goal_pose`: a single volatile publish from a transient
# `docker exec` node races DDS discovery and, on a cold CI runner, is dropped
# before bt_navigator's /goal_pose subscription is matched - so the goal never
# arrives, nav2 never drives, and no fault is ever raised. An action client
# waits for the server and confirms the goal is accepted, guaranteeing delivery.
# It returns as soon as the goal is ACCEPTED (it does not block on the drive
# result), so a healthy-resume goal does not stall the caller.
set -eu
X="${1:-1.5}"; Y="${2:-1.0}"

# ros2 is not on the container's default PATH and `docker exec` does not run the
# image entrypoint that sources it, so source the ROS overlay inside the exec.
# Goal coordinates are passed as env vars (not string-interpolated into the
# heredoc) so the Python body stays a fixed, quote-safe literal.
docker exec -e GOAL_X="$X" -e GOAL_Y="$Y" -i ota_demo_gateway bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && python3 -' <<'PYEOF'
import os
import sys

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

x = float(os.environ["GOAL_X"])
y = float(os.environ["GOAL_Y"])

rclpy.init()
node = rclpy.create_node("send_goal_cli")
client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
if not client.wait_for_server(timeout_sec=30.0):
    print("send-goal: /navigate_to_pose action server not available within 30s", file=sys.stderr)
    sys.exit(1)

goal = NavigateToPose.Goal()
goal.pose.header.frame_id = "map"
goal.pose.pose.position.x = x
goal.pose.pose.position.y = y
goal.pose.pose.orientation.w = 1.0

send_future = client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, send_future)
handle = send_future.result()
if handle is None or not handle.accepted:
    print("send-goal: goal was rejected by /navigate_to_pose", file=sys.stderr)
    sys.exit(1)

rclpy.shutdown()
PYEOF
echo "Goal sent: (${X}, ${Y})"
