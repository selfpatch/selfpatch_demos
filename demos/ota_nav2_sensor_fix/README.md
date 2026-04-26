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
# 1. Build artifacts (compiles fixed_lidar + obstacle_classifier_v2,
#    generates catalog.json + tarballs).
./scripts/build_artifacts.sh

# 2. Start the stack (gateway + plugin + demo nodes + update server).
docker compose up --build
```

In another terminal, drive the demo:

```bash
# List the registered updates.
curl -s http://localhost:8080/api/v1/updates | jq '.[].id'

# Run an update: prepare downloads the artifact, execute swaps + restarts.
curl -X PUT http://localhost:8080/api/v1/updates/fixed_lidar_2_1_0/prepare
curl -X PUT http://localhost:8080/api/v1/updates/fixed_lidar_2_1_0/execute

# Install a new package.
curl -X PUT http://localhost:8080/api/v1/updates/obstacle_classifier_v2_1_0_0/prepare
curl -X PUT http://localhost:8080/api/v1/updates/obstacle_classifier_v2_1_0_0/execute

# Uninstall a deprecated one.
curl -X PUT http://localhost:8080/api/v1/updates/broken_lidar_legacy_remove/execute
```

Tear down: `docker compose down`.

## Adding a Foxglove visualization

Install the `ros2_medkit_foxglove_extension` (which now ships an Updates
panel - see https://github.com/selfpatch/ros2_medkit_foxglove_extension)
in your local Foxglove Studio, then point it at
`http://localhost:8080/api/v1`. The Updates panel exposes Prepare and
Execute buttons next to each catalog entry.

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
