#!/bin/bash
# On-failure diagnostics for the ota_nav2_sensor_fix CI jobs.
#
# The gateway container multiplexes ~15 nodes (headless gz + the full Nav2 stack
# + the gateway + the log/action bridges) into a single stdout, so
# `docker compose logs gateway --tail=N` shows only the last few seconds of
# discovery chatter and never the nav2 goal handling that actually explains a
# failure. This dumps the FULL compose log plus live ROS introspection
# (lifecycle states, the /navigate_to_pose action server, /scan, /amcl_pose,
# the map->odom TF, controllers, processes, faults) so a red run is debuggable.
#
# Best-effort: every probe is bounded and this script never fails the CI step.
# Usage: ./tests/dump_ota_diagnostics.sh [output_log_path]
set +e

DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../demos/ota_nav2_sensor_fix" && pwd)"
C="${OTA_DEMO_GATEWAY_CONTAINER:-ota_demo_gateway}"
OUT="${1:-ota_diagnostics.log}"

# Run a command inside the gateway container with the ROS overlay sourced
# (docker exec does not run the image entrypoint that sources it).
RUN() { docker exec "$C" bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && $*" 2>&1; }

{
    echo "===== full compose logs ====="
    ( cd "$DEMO_DIR" && docker compose logs --no-color --timestamps )
    echo
    echo "===== ros2 node list ====="
    RUN "timeout 15 ros2 node list | sort"
    echo "===== nav2 lifecycle (is the stack active?) ====="
    for n in map_server amcl planner_server controller_server bt_navigator; do
        printf '%s: ' "$n"
        RUN "timeout 8 ros2 lifecycle get /$n"
    done
    echo "===== /navigate_to_pose action server (did the goal have a server?) ====="
    RUN "timeout 12 ros2 action info /navigate_to_pose -t"
    echo "===== /amcl_pose (is the robot localized?) ====="
    RUN "timeout 8 ros2 topic echo --once /amcl_pose"
    echo "===== /scan header (is the lidar publishing?) ====="
    RUN "timeout 8 ros2 topic echo --once --field header /scan"
    echo "===== map->odom TF (localization connected?) ====="
    RUN "timeout 8 ros2 run tf2_ros tf2_echo map odom"
    echo "===== controllers ====="
    RUN "timeout 10 ros2 control list_controllers"
    echo "===== live processes ====="
    docker exec "$C" ps -eo pid,etimes,args 2>&1 | grep -iE "gz|nav2|controller|amcl|bt_navigator|lidar_node|bridge" | grep -v grep
    echo "===== faults (bt-navigator / controller-server) ====="
    docker exec "$C" curl -s localhost:8080/api/v1/apps/bt-navigator/faults 2>&1
    echo
    docker exec "$C" curl -s localhost:8080/api/v1/apps/controller-server/faults 2>&1
    echo
} | tee "$OUT"
