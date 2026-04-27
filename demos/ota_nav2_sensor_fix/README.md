# OTA over SOVD - nav2 sensor fix demo

End-to-end demo: a `ros2_medkit` gateway with a dev-grade OTA plugin that
demonstrates the full Update / Install / Uninstall lifecycle on ROS 2 nodes
without SSH-ing into the robot.

## What this shows

Three things you can do to a ROS 2 robot over the air:

1. **Update** - swap a running sensor node with a fixed version (the
   `broken_lidar` -> `fixed_lidar` flip).
2. **Install** - pull and start a new ROS 2 package
   (`obstacle_classifier_v2`).
3. **Uninstall** - stop and remove a deprecated package
   (`broken_lidar_legacy`).

All three operations are SOVD ISO 17978-3 compliant - the kind is derived
from `updated_components` / `added_components` / `removed_components` in the
update package metadata.

## Quickstart

```bash
# Build artifacts + start gateway, plugin, demo nodes, update server.
./run-demo.sh
```

The first run pulls `ros:jazzy` and builds the gateway from source - takes
~10 minutes. Subsequent runs reuse the layer cache.

In another terminal, drive the demo:

```bash
./check-demo.sh           # show registered updates + live process state
./trigger-update.sh       # broken_lidar -> fixed_lidar (the headline scene)
./trigger-install.sh      # install obstacle_classifier_v2 from scratch
./trigger-uninstall.sh    # remove broken_lidar_legacy
./stop-demo.sh            # tear down
```

Each trigger script issues SOVD `PUT /updates/{id}/prepare` then `/execute`
and prints the resulting status plus the live process list.

If host port 8080 is taken, override with `OTA_GATEWAY_PORT=8081 ./run-demo.sh`.

Tear down: `docker compose down`.

## Foxglove Studio visualization

The gateway container bakes in a TurtleBot3 burger + Nav2 stack running on
top of headless Gazebo. `foxglove_bridge` runs on port `8765` and exposes
the full topic set: `/tf`, `/tf_static`, `/scan`, `/odom`, `/map`,
`/cmd_vel`, `/global_costmap/costmap`, `/local_costmap/costmap`, etc. - so a
Foxglove **3D** panel renders the actual robot in the world out of the box.

1. Open Foxglove Studio -> **Open connection** -> **Foxglove WebSocket** ->
   `ws://localhost:8765`. The Topics panel should list all of the topics
   above.
2. Drop in a **3D** panel. You should see the TB3 burger sitting in the
   default `turtlebot3_world.world` map, with the laser scan cone visible.
   Before the OTA update, ray index 180 reports a phantom 1 m return - the
   "obstacle" the demo's narrative pivots on.
3. Install the [`ros2_medkit_foxglove_extension`](https://github.com/selfpatch/ros2_medkit_foxglove_extension)
   (`npm run local-install` in that repo, or drag-and-drop the `.foxe`
   onto Foxglove). It ships three panels: Entity Browser, Faults Dashboard,
   and **ros2_medkit Updates**.
4. Add the **ros2_medkit Updates** panel and set its `baseUrl` to
   `http://localhost:8080/api/v1` (or the port you picked via
   `OTA_GATEWAY_PORT`). Click **Prepare** then **Execute** for
   `fixed_lidar_2_1_0`. The 3D panel should show the phantom return
   disappearing as `broken_lidar` is killed and `fixed_lidar` starts.

### Driving the robot to make the narrative reproducible

The demo doesn't auto-publish a navigation goal - that keeps it deterministic
for CI. To trigger the "robot stuck on phantom obstacle" beat manually:

```bash
# From the host (or any container on the same ROS_DOMAIN_ID=42):
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
```

Foxglove's **3D** panel also has a built-in "Publish" tool - select pose
mode, click a point ahead of the robot, and Foxglove publishes `/goal_pose`
for you. Before the update, Nav2 refuses to drive through the phantom return;
after `trigger-update.sh`, the robot completes the goal.

## Disclosures

This is **dev-grade** OTA. Deliberately missing for production:

- No artifact signing or signature verification
- No atomic swap (in-place overwrite)
- No A/B partition rollout
- No fleet-wide staged rollout
- No persistent update state across gateway restarts
- No automated health-gated rollback policy
- No audit log

Perfect for: prototypes, lab robots, internal demos, dev environments.

For production-grade OTA (rollout safety, signing, A/B partitions,
fleet-aware staging), reach out.
