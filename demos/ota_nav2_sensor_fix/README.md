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

The gateway container also runs `foxglove_bridge` on port `8765` so
Foxglove Studio can subscribe to ROS 2 topics (e.g. `/scan` from
broken_lidar / fixed_lidar).

1. Open Foxglove Studio -> **Open connection** -> **Foxglove WebSocket** ->
   `ws://localhost:8765`. You should see `/scan` and other topics in the
   Topics panel.
2. Install the [`ros2_medkit_foxglove_extension`](https://github.com/selfpatch/ros2_medkit_foxglove_extension)
   (`npm run local-install` in that repo, or drag-and-drop the `.foxe`
   onto Foxglove). It ships three panels: Entity Browser, Faults Dashboard,
   and **ros2_medkit Updates**.
3. Add the **ros2_medkit Updates** panel and set its `baseUrl` to
   `http://localhost:8080/api/v1` (or the port you picked via
   `OTA_GATEWAY_PORT`).
4. Click **Prepare** and **Execute** in the Updates panel - the same SOVD
   endpoints `trigger-update.sh` hits, with progress feedback in the panel
   and live `/scan` updates in the 3D scene.

## Adding nav2 / a sim

This demo intentionally omits a nav2 sim from the compose so the stack stays
small and reliably reproducible. To make the visual story complete:

- Bring up your favourite turtlebot3 sim (`turtlebot3_gazebo`) and point it
  at `ROS_DOMAIN_ID=42` to share the DDS namespace with the gateway.
- The broken_lidar node publishes a phantom return on `/scan` ~1m straight
  ahead. nav2's costmap will trace it as an obstacle and the planner will
  refuse to drive forward. After the update flow, fixed_lidar publishes a
  clean scan and the path planner unblocks.

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
