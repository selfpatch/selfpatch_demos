# OTA over SOVD - nav2 sensor fix demo

End-to-end demo: a `ros2_medkit` gateway with a dev-grade OTA plugin that
demonstrates a real update / publish-a-hotfix loop on a ROS 2 node without
SSH-ing into the robot.

## What this shows

The headline scene is a diagnostic loop, not just a button-press update:

1. The robot boots on the known-good lidar (`fixed_lidar` running as
   `scan_sensor_node`) - clean `/scan`, no fault.
2. At container startup a routine-looking software update
   (`broken_lidar_3_0_0`) is **auto-applied** - the entrypoint swaps
   `scan_sensor_node` over to `broken_lidar` before the mission even
   starts. This is the root cause the operator will have to find later.
3. An operator sends a nav goal with `./send-goal.sh`. The robot drives,
   and the operator's Foxglove view drops (narrative: the operator is away
   and the viewer link is lost for a few minutes). Only the operator's view
   goes dark - the robot, the on-robot gateway, and the fault manager keep
   running and capturing the whole time.
4. While driving, `broken_lidar` overlays a blocking phantom sector onto
   the real `/scan_sim` data straight ahead - real obstacles elsewhere in
   the scan stay visible. Nav2 genuinely cannot make progress:
   `controller_server` logs "Failed to make progress" and
   `navigate_to_pose` aborts. Two generic `ros2_medkit` bridges (not a
   custom fault in the scan node) turn Nav2's own failure into SOVD
   faults: `ros2_medkit_action_status_bridge` reports
   `ACTION_NAVIGATE_TO_POSE_ABORTED` on `bt-navigator` (the headline
   fault) and `ros2_medkit_log_bridge` reports a
   `LOG_CONTROLLER_SERVER_*` fault on `controller-server` (supporting).
   `ros2_medkit_fault_manager` confirms both immediately, captures a
   freeze-frame + MCAP recording under the `bt-navigator` fault's
   `environment_data.snapshots`, and **latches** it - no self-healing.
5. The operator reconnects: the robot is stopped, and the fault is red
   (latched).
6. The operator downloads the MCAP capture
   (`GET /apps/bt-navigator/bulk-data/rosbags/ACTION_NAVIGATE_TO_POSE_ABORTED`)
   and opens it in Foxglove - the phantom blocking the path is visible in
   the replayed `/scan`. The Faults Dashboard panel's freeze-frame shows
   the same state at confirmation time.
   To confirm the root cause rather than guess, the operator runs the
   `health-check` app's single `run_health_checks` operation from the
   Operations tab (all four checks default to enabled): localization and
   drivetrain come back healthy, but the report shows lidar failed with a
   stuck sector and costmap flags an obstacle ahead that is not in the map -
   it is the lidar, not something downstream.
7. `GET /api/v1/updates` shows only `broken_lidar_3_0_0` - the suspect
   recent change, and the fix the operator needs is not registered yet.
8. The operator publishes the hotfix with `./publish-fix.sh` (SOVD
   `POST /api/v1/updates`, registering `fixed_lidar_3_0_1` - a **forward**
   fix, version 3.0.1 > the bad 3.0.0, not a rollback to a previous build).
   `GET /api/v1/updates` now shows both `broken_lidar_3_0_0` and
   `fixed_lidar_3_0_1`.
9. The operator applies the fix with `./apply-fix.sh` (prepare + execute
   `fixed_lidar_3_0_1`) - `scan_sensor_node` swaps from `broken_lidar` to
   `fixed_lidar` and `/scan` is clean again. The fault stays **latched** -
   applying the fix does not clear the DTC.
10. The operator clears the faults with `./clear-fault.sh` (an explicit
    `DELETE /apps/bt-navigator/faults/ACTION_NAVIGATE_TO_POSE_ABORTED` plus
    a clear-all `DELETE /apps/controller-server/faults` for the
    content-hashed `LOG_*` code) - the Faults Dashboard goes green.
11. The operator resumes the mission with `./send-goal.sh` - the robot
    reaches the goal.

The update is SOVD ISO 17978-3 compliant - the kind is derived from
`updated_components` in the update package metadata.

## Quickstart

```bash
# Build artifacts + start gateway, plugin, demo nodes, update server.
./run-demo.sh
```

The first run pulls `ros:jazzy`, installs the Nav2 + gz-sim runtime, clones
the Robotnik RB-Theron + AWS small-warehouse assets (~3 GB) and builds the
gateway from source - takes ~15-20 minutes on a fresh cache. Subsequent runs
reuse the layer cache.

In another terminal, drive the demo:

```bash
./check-demo.sh           # at a glance: scan node, applied updates, faults
./send-goal.sh            # send a nav goal (mission start / resume)
./publish-fix.sh          # register fixed_lidar_3_0_1 (SOVD POST /updates) - not in the boot catalog
./apply-fix.sh            # broken_lidar -> fixed_lidar_3_0_1 (prepare + execute the published fix)
./clear-fault.sh          # operator clear of the latched bt-navigator/controller-server faults
./trigger-bad-update.sh   # re-arm broken_lidar_3_0_0 (normally auto-applied at boot)
./stop-demo.sh            # tear down
```

`publish-fix.sh` issues a SOVD `POST /updates` to register the held-back
`fixed_lidar_3_0_1` hotfix. `apply-fix.sh` and `trigger-bad-update.sh` issue
SOVD `PUT /updates/{id}/prepare` then `/execute` and print the resulting
status plus the live process list; `apply-fix.sh` guards on the fix being
registered first and tells you to run `./publish-fix.sh` if it isn't.
`clear-fault.sh` issues a plain SOVD
`DELETE /apps/bt-navigator/faults/ACTION_NAVIGATE_TO_POSE_ABORTED` plus a
clear-all `DELETE /apps/controller-server/faults` (the `LOG_*` code there
is content-hashed, so it is cleared by entity rather than by exact code).
`send-goal.sh` sends the goal through the `/navigate_to_pose` action inside the
gateway container (an action client that waits for the server and confirms the
goal is accepted, so a transient publisher never drops it before nav2 sees it).

Port overrides (set as env vars before `./run-demo.sh`):

- `OTA_GATEWAY_PORT` - gateway HTTP API (default `8080`)
- `OTA_FOXGLOVE_BRIDGE_PORT` - foxglove_bridge WebSocket (default `8765`)

Tear down: `docker compose down`.

## Diagnosing the incident over SOVD

Everything in the loop above is also doable with plain `curl` - the
Foxglove panels (next section) are a convenience layer on top of the same
SOVD REST calls.

```bash
API=http://localhost:8080/api/v1

# 1. Which update is applied to scan_sensor_node right now - the suspect
#    recent change. Only the bad update is in the boot catalog; the fix
#    is not registered yet.
curl -s "${API}/updates" | jq -r '.items[]'
curl -s "${API}/updates/broken_lidar_3_0_0/status" | jq .

# 2. Is ACTION_NAVIGATE_TO_POSE_ABORTED confirmed on bt-navigator (the
#    headline fault)? Also check controller-server for the supporting
#    LOG_* fault. (default filter = PREFAILED + CONFIRMED)
curl -s "${API}/apps/bt-navigator/faults" | jq .
curl -s "${API}/apps/controller-server/faults" | jq .

# 3. Fault detail - freeze-frame + the MCAP link live under
#    environment_data.snapshots.
curl -s "${API}/apps/bt-navigator/faults/ACTION_NAVIGATE_TO_POSE_ABORTED" | jq .

# 4. Download the MCAP recording and open it in Foxglove.
curl -O -J "${API}/apps/bt-navigator/bulk-data/rosbags/ACTION_NAVIGATE_TO_POSE_ABORTED"

# 4b. Confirm the root cause with the single run_health_checks operation
#     (all four checks default to enabled). localization + drivetrain come
#     back healthy; the report line for lidar shows a stuck sector.
curl -s -X POST -H 'Content-Type: application/json' -d '{}' \
  "${API}/apps/health-check/operations/run_health_checks/executions" | jq '.parameters // .'

# 5. Publish the forward hotfix (or use ./publish-fix.sh). It is not in the
#    boot catalog - you register it by POSTing its descriptor, a JSON you
#    provide by hand (see updates/README.md for the fields), via SOVD
#    POST /updates.
curl -fsS -X POST -H 'Content-Type: application/json' \
  -d @updates/fixed_lidar_3_0_1.json "${API}/updates"
curl -s "${API}/updates" | jq -r '.items[]'

# 6. Apply the published fix (or use ./apply-fix.sh).
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
  "${API}/updates/fixed_lidar_3_0_1/prepare"
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
  "${API}/updates/fixed_lidar_3_0_1/execute"

# 7. Both faults are still latched after applying the fix - clear them
#    explicitly (or use ./clear-fault.sh). The controller-server LOG_*
#    code is content-hashed, so clear it with a clear-all on the entity
#    instead of a hardcoded code.
curl -X DELETE "${API}/apps/bt-navigator/faults/ACTION_NAVIGATE_TO_POSE_ABORTED"
curl -X DELETE "${API}/apps/controller-server/faults"

# 8. Resume the mission (or use ./send-goal.sh).
```

## Foxglove Studio visualization

The gateway container bakes in a Robotnik RB-Theron AMR + Nav2 stack running
on top of headless Gazebo in the AWS small-warehouse world. `foxglove_bridge`
runs on port `8765` and exposes the full topic set: `/tf`, `/tf_static`,
`/scan`, `/odom`, `/map`, `/cmd_vel`, `/global_costmap/costmap`,
`/local_costmap/costmap`, etc. - so a Foxglove **3D** panel renders the actual
robot in the warehouse out of the box.

1. Open Foxglove Studio -> **Open connection** -> **Foxglove WebSocket** ->
   `ws://localhost:8765`. The Topics panel should list all of the topics
   above.
2. Drop in a **3D** panel. In its settings set **Scene -> Mesh up axis ->
   Z**, then reload (Ctrl-R). The mesh geometry carries no up-axis metadata so
   Foxglove defaults to Y-up, which renders the robot rotated with its parts
   scattered; Z-up puts it upright and assembled. You should then see the
   RB-Theron sitting in the AWS small-warehouse world.
   Shortly after boot, the auto-applied `broken_lidar_3_0_0` update swaps
   `scan_sensor_node` over to `broken_lidar` - a forward sector of `/scan`
   starts reporting a phantom close return (a stuck lidar sector). Nav2 cannot
   get past the phantom, so the robot stalls and `navigate_to_pose` aborts -
   the failure the demo's narrative pivots on.
3. Install the [`ros2_medkit_foxglove_extension`](https://github.com/selfpatch/ros2_medkit_foxglove_extension)
   (`npm run local-install` in that repo, or drag-and-drop the `.foxe`
   onto Foxglove). It ships three panels: Entity Browser, Faults Dashboard,
   and **ros2_medkit Updates**.
4. Add the **Faults Dashboard** panel. Once the robot stalls at the
   phantom (see "Driving the robot" below), `ACTION_NAVIGATE_TO_POSE_ABORTED`
   shows up CONFIRMED on `bt-navigator` (the headline fault) alongside a
   `LOG_CONTROLLER_SERVER_*` fault on `controller-server` (supporting), and
   both stay latched. Expand the `bt-navigator` fault to see the
   freeze-frame snapshot (`/scan`, `/cmd_vel`, `/local_costmap/costmap`)
   captured at confirmation, plus the downloadable MCAP recording.
5. Add the **ros2_medkit Updates** panel and set its `baseUrl` to
   `http://localhost:8080/api/v1` (or the port you picked via
   `OTA_GATEWAY_PORT`). `broken_lidar_3_0_0` shows as the update applied
   to `scan_sensor_node` - the fix is not registered yet. Run
   `./publish-fix.sh` in a terminal to register `fixed_lidar_3_0_1`
   (SOVD `POST /updates`); it then appears in the panel. Click **Prepare**
   then **Execute** for `fixed_lidar_3_0_1` to apply it - the 3D panel
   should show the phantom return disappearing as `broken_lidar` is killed
   and `fixed_lidar` starts. The Faults Dashboard entry stays red until you
   also clear it.

### Driving the robot to make the narrative reproducible

The demo doesn't auto-publish a navigation goal - that keeps it
deterministic for CI. Use `./send-goal.sh` to drive the loop yourself:

```bash
./send-goal.sh             # defaults to (1.5, 1.0); pass x y to override
```

`send-goal.sh` `docker exec`s into the gateway container (sourcing the ROS
overlay itself, since `docker exec` skips the image entrypoint) and sends the
goal through the `/navigate_to_pose` action. Foxglove's **3D** panel
also has a built-in "Publish" tool - select pose mode, click a point ahead
of the robot, and Foxglove publishes `/goal_pose` for you.

While `broken_lidar_3_0_0` is applied (the boot default), driving toward
the phantom blocks the path and stalls Nav2 - `navigate_to_pose` aborts
and confirms `ACTION_NAVIGATE_TO_POSE_ABORTED` on `bt-navigator` (plus a
supporting `LOG_*` fault on `controller-server`). Watch the Faults
Dashboard panel or poll `GET /apps/bt-navigator/faults` and
`GET /apps/controller-server/faults`. After `./publish-fix.sh`,
`./apply-fix.sh`, and `./clear-fault.sh`, send the goal again and the robot
reaches it with a clean `/scan`.

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
