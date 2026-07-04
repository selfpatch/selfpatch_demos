#!/bin/bash
# Publish a Nav2 goal into the demo container (ROS_DOMAIN_ID=42). Used to
# start the mission (robot drives into the phantom) and to resume after fix.
set -eu
X="${1:-1.5}"; Y="${2:-1.0}"
GOAL="{header: {frame_id: map}, pose: {position: {x: ${X}, y: ${Y}, z: 0.0}, orientation: {w: 1.0}}}"
# ros2 is not on the container's default PATH and `docker exec` does not run the
# image entrypoint that sources it, so source the ROS overlay inside the exec.
docker exec ota_demo_gateway bash -c \
  "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped '${GOAL}'"
echo "Goal sent: (${X}, ${Y})"
