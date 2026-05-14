# manymove_industrial

manymove BT manipulator pipeline (xArm7 fake hardware) with `ros2_medkit`
fault reporting. The image follows the manymove project's own docker style
(manymove + Groot + xarm_ros2 all built from source) and adds the medkit
fault_manager / gateway / web UI on top.

## What's in the demo

- xArm7 cobot under `fake_components` hardware (no real arm needed)
- MoveIt 2 + manymove BT pipeline launched via the upstream
  `xarm7_movegroup_fake_cpp_trees.launch.py`
- `bt_client_xarm7` from the [selfpatch fork](https://github.com/selfpatch/manymove)
  emits `MANYMOVE_*` fault codes through `ros2_medkit_fault_reporter` on
  collision / retry / IO failures
- Groot BT visualizer (built from source, same as the upstream demo) for a
  live view of the BT during the run
- manymove Qt HMI for operator-driven blackboard mutation
- medkit fault_manager + gateway + Web UI on `:3000` for the fault timeline

## Status

**v1 (this directory):** real BT pipeline + medkit. Inject scripts use the
`update_blackboard` HMI service so faults are produced organically by the BT
nodes (not synthesised on the FaultManager service).

**v2 (this directory, since `feat/manymove-industrial`):** adds the PLC tier.
A Python OPC UA server (`plc_sim/`) emulates the line PLC (photoeye,
conveyor, e-stop tags) and publishes `AlarmConditionType` events. A ROS 2
bridge (`opcua_bridge/`) subscribes to those events and forwards them to
the `FaultManager` as `MANYMOVE_PLC_*` faults with `source_id=/plc/sensor_io`.
The PLC sim is swappable with a real OpenPLC v3 + ST program later; the
OPC UA AlarmConditionType surface stays the same.

This gives the demo a real cross-source narrative: a PLC photoeye flicker
WARN can land in the same dashboard alongside a manymove BT
`MANYMOVE_PLANNER_COLLISION_DETECTED` CRITICAL, on one timeline.

## Prerequisites

- Docker Engine + `docker compose` plugin
- `xhost` (any standard X server install) for the GUI tools on the cpu profile
- `curl`, `jq` for the host-side helper scripts

The first build is heavy (downloads + colcon-builds manymove, Groot and
xarm_ros2 from source). Expect 25-40 minutes the first time; subsequent
builds reuse Docker layer cache.

## Quick start

```bash
./run-demo.sh           # cpu profile, X11 RViz/Groot/HMI, web UI on :3000
./check-demo.sh         # gateway health + entity discovery + fault list
./stop-demo.sh

./run-demo.sh --ci      # headless, no web UI / no X11 (matches CI smoke job)
```

## Fault scenarios

Container scripts under `container_scripts/manymove-planning/` are mounted
at the well-known gateway scripts dir and runnable from the web UI or via
`scripts-api.sh`:

- `arm-self-test` - emits `MANYMOVE_SELFTEST` (INFO + immediate PASSED) so
  operators can confirm the medkit pipeline is wired without producing a
  real fault entry.
- `inject-collision` - flips the BT blackboard `collision_detected` flag.
  The next `MoveManipulatorAction::onStart` tick observes it, emits
  `MANYMOVE_PLANNER_COLLISION_DETECTED` (ERROR) and returns FAILURE.
  This is a real BT fault, not a synthesised report.
- `restore-normal` - clears `collision_detected`, `stop_execution` and
  triggers `reset` + `start` on the blackboard so the BT picks up cleanly.

## SOVD manifest

`config/manymove_industrial_manifest.yaml` declares the BT client, MoveIt
move_group, manymove planner and object_manager action servers, the
ufactory_driver, the HMI service node, and the medkit fault_manager /
gateway. Apps' `ros_binding.namespace + node_name` link the gateway to the
runtime FQNs from the upstream xarm7 launch (the manymove fork renames
each `bt_client_*` executable's ROS node to match its binary).

## Live BT view (Groot)

The image builds Groot from source (matching upstream manymove). Once the
demo is up:

```bash
docker compose exec manymove-sim bash -lc \
  "source install/setup.bash && src/Groot/build/Groot"
```

This is the same workflow upstream manymove uses to record demo videos.

## TODO: PLC tier (v1.5)

Tier 2 PLC correlation narrative is deferred to a follow-up commit:

- Custom OpenPLC v3 image (mirror `demos/openplc_tank/openplc/Dockerfile`,
  enable OPC UA driver) + ST program (conveyor + cylinder + photoeye + e-stop).
- Python `asyncua` ROS<->OPC UA bridge node that publishes PLC tags as ROS
  topics, exposes `/plc/set_estop`, and emits its own `PLC_ESTOP_ENGAGED`
  fault.
- Correlation logic: bridge subscribes to `/fault_manager/events`, observes
  the pair (`PLC_ESTOP_ENGAGED` + `MANYMOVE_PLANNER_ESTOP_TRIGGERED`) within
  a 2 s window, emits the synthetic `CORRELATED_LINE_ESTOP` fault. Filters
  events whose source_id matches its own to avoid feedback.
- `inject-hard-fault.sh` flips the PLC e-stop, demonstrating the tier-2
  narrative end to end.
