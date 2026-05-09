# manymove_industrial

manymove BehaviorTree.CPP manipulator pipeline with `ros2_medkit` fault
reporting. The demo brings up a medkit gateway against a SOVD manifest
describing the manymove BT client, MoveIt move_group, and the medkit
fault management stack, and provides three container scripts that exercise
the fault pipeline end-to-end.

## Status

**v1 (this directory):** medkit gateway + fault_manager wired against the
SOVD manifest, container scripts that drive `MANYMOVE_*` fault codes via
the `/fault_manager/report_fault` service. The actual manymove BT client
and `move_group` are not yet launched here; see "TODO" below.

**v1.5 (planned):** add an OpenPLC v3 service (custom Dockerfile mirroring
`openplc_tank`), an `asyncua`-based ROS<->OPC UA bridge, and a tier-2
correlation rule (`PLC_ESTOP_ENGAGED` + `MANYMOVE_PLANNER_ESTOP_TRIGGERED`
within a 2 s window -> synthetic `CORRELATED_LINE_ESTOP`).

## Prerequisites

- Docker Engine + `docker compose` plugin
- `curl`, `jq` for the host-side helper scripts

The image pulls the medkit deb packages
(`ros-jazzy-ros2-medkit-fault-reporter`, `…-fault-manager`, `…-gateway`,
`…-msgs`, `…-graph-provider`, `…-diagnostic-bridge`, `…-cmake`) at build
time, so no medkit clone is needed.

## Quick start

```bash
./run-demo.sh           # cpu profile, web UI on :3000
./check-demo.sh         # gateway health + entity discovery + fault list
./stop-demo.sh

./run-demo.sh --ci      # headless, no web UI (matches CI smoke job)
```

## Fault scenarios

Three container scripts under `container_scripts/manymove-planning/` are
registered with the gateway and runnable from the web UI or with
`scripts-api.sh`:

- `arm-self-test` - emits `MANYMOVE_SELFTEST` (INFO + immediate PASSED) so
  operators can confirm the medkit pipeline is wired without producing a
  real fault entry.
- `inject-collision` - reports `MANYMOVE_PLANNER_COLLISION_DETECTED` (ERROR)
  with `source_id=/bt_client_xarm7`. Mirrors the collision branch instrumented
  in `manymove_cpp_trees::MoveManipulatorAction::onStart`.
- `restore-normal` - sends EVENT_PASSED for the codes most likely to have
  fired, clearing their LocalFilter state.

## SOVD manifest

`config/manymove_industrial_manifest.yaml`. Apps' `ros_binding.namespace +
node_name` link the gateway to runtime FQNs from the manymove fork (which
renames each `bt_client_*` executable's ROS node to match its binary).

## TODO

- Launch the manymove BT client and `move_group` so the codes from
  `docs/FAULT_CODES.md` fire from real BT execution rather than the inject
  scripts. Blocked on either packaging xarm-ros2 for jazzy or producing a
  Panda variant of `bt_client_*.cpp`.
- Add OpenPLC + OPC UA bridge for the tier-2 correlation narrative
  (`PLC_ESTOP_ENGAGED` + `MANYMOVE_PLANNER_ESTOP_TRIGGERED` -> synthetic
  `CORRELATED_LINE_ESTOP`).
- Add `inject-soft-fault.sh` that drives `MANYMOVE_PLANNER_RETRY_ATTEMPT`
  per BT tick, exercising LocalFilter throttling.
