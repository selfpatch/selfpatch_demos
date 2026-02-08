# MoveIt 2 Pick-and-Place Integration Demo

A comprehensive integration demo combining a **Panda 7-DOF robot arm** with **MoveIt 2** motion planning and **ros2_medkit** SOVD-compliant diagnostics. The robot performs continuous pick-and-place cycles while a manipulation monitor detects faults — planning failures, controller timeouts, joint limit violations — and reports them through the SOVD REST API.

## Status

✅ **Demo Ready** — Docker-based deployment with MoveIt 2, RViz visualization, mock hardware, and full ros2_medkit stack.

## Overview

This demo demonstrates:

- **MoveIt 2 motion planning** with the Panda 7-DOF arm and gripper
- **Continuous pick-and-place** loop as a realistic manipulation workload
- **Manipulation fault monitoring** (planning failures, trajectory errors, joint limits)
- **SOVD-compliant REST API** with Areas → Components → Apps → Functions hierarchy
- **Manifest-based entity discovery** (hybrid mode with runtime enrichment)
- **5 fault injection scenarios** with one-click scripts
- **Web UI** for visual entity browsing and fault monitoring

## Prerequisites

- Docker and docker-compose
- X11 display server (for RViz GUI) or `--headless` mode
- (Optional) NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- ~7 GB disk space for Docker image

## Quick Start

### 1. Start the Demo

```bash
cd demos/moveit_pick_place
./run-demo.sh
```

That's it! The script will:
1. Build the Docker image (first run: ~15-20 min, ~7 GB)
2. Set up X11 forwarding for RViz GUI
3. Launch Panda robot + MoveIt 2 + ros2_medkit gateway
4. Launch sovd_web_ui at http://localhost:3000

**REST API:** http://localhost:8080/api/v1/
**Web UI:** http://localhost:3000/

### 2. Available Options

```bash
./run-demo.sh                  # Default: Gazebo simulation, daemon mode
./run-demo.sh --fake           # Fake hardware (mock controllers, no physics)
./run-demo.sh --nvidia         # Gazebo + GPU acceleration
./run-demo.sh --fake --nvidia  # Fake hardware + GPU acceleration
./run-demo.sh --headless       # No GUI (CI/server)
./run-demo.sh --attached       # Foreground with logs
./run-demo.sh --no-cache       # Rebuild without cache
./run-demo.sh --update         # Pull latest images first
```

**Simulation modes:**
- **Default (Gazebo)** — Gazebo Harmonic physics simulation with `gz_ros2_control`. Realistic dynamics, 3D world view. Slower startup (~30s), needs X11 or `--headless`. Recommended with `--nvidia` for GPU acceleration.
- **Fake hardware (`--fake`)** — Mock controllers echo commanded positions instantly. Fast startup (~10s), works headless, no physics. Good for diagnostics testing.

### 3. Moving the Arm

Use the interactive arm controller to send joint trajectories:

```bash
./move-arm.sh              # Interactive menu
./move-arm.sh ready        # Go to ready pose
./move-arm.sh pick         # Go to pick pose
./move-arm.sh demo         # Run full pick → place → home cycle
```

The script sends goals directly to the `panda_arm_controller/follow_joint_trajectory` action.
It works both from outside (via `docker exec`) and from inside the container.

### 4. Viewing Logs

```bash
docker compose --profile cpu logs -f         # CPU version
docker compose --profile nvidia logs -f      # NVIDIA version
docker exec -it moveit_medkit_demo bash      # Shell into container
```

### 5. Stopping the Demo

```bash
./stop-demo.sh                 # Stop containers
./stop-demo.sh --volumes       # Stop and remove volumes
./stop-demo.sh --images        # Stop and remove images
```

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Docker Container                              │
│                                                                  │
│  ┌──────────┐  ┌──────────────┐  ┌──────────────────────────┐  │
│  │ Fake HW  │  │  ros2_control │  │     MoveIt 2             │  │
│  │  (mock   │──│  Controllers  │──│  move_group (OMPL)       │  │
│  │ controllers)│  arm+gripper  │  │                          │  │
│  └──────────┘  └──────────────┘  └──────────┬───────────────┘  │
│                                              │                   │
│  ┌──────────────────────────────┐            │                   │
│  │  pick_place_loop.py          │◄───────────┘                   │
│  │  (continuous manipulation)   │   MoveGroup action             │
│  └──────────────────────────────┘                                │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                   ros2_medkit Stack                        │   │
│  │                                                           │   │
│  │  manipulation_monitor.py ──► fault_manager ◄── diag_bridge│   │
│  │         │                        │                        │   │
│  │         │ monitors:              │ stores faults           │   │
│  │         │ • /move_action status  │                        │   │
│  │         │ • /controller status   ▼                        │   │
│  │         │ • /joint_states    gateway_node ──► REST API    │   │
│  └─────────┴────────────────────────┬───────────────────────┘   │
│                                     │ :8080                      │
└─────────────────────────────────────┼────────────────────────────┘
                                      │
                        ┌─────────────┼─────────────┐
                        │             │             │
                   sovd_web_ui    curl/httpie    MCP Server
                    :3000                        (LLM tools)
```

## Entity Hierarchy (SOVD)

```
Areas
├── manipulation/        — Robot arm and gripper hardware
│   Components
│   ├── panda-arm        — 7-DOF Franka Emika Panda
│   │   Apps: joint-state-broadcaster, panda-arm-controller, robot-state-publisher
│   └── panda-gripper    — 2-finger parallel gripper
│       Apps: panda-hand-controller
│
├── planning/            — MoveIt 2 motion planning stack
│   Components
│   ├── moveit-planning  — OMPL planning pipeline
│   │   Apps: move-group
│   └── pick-place-loop — Pick-and-place demo node
│       Apps: pick-place-node
│
├── diagnostics/         — ros2_medkit gateway and fault management
│   Components
│   ├── gateway          — REST API
│   │   Apps: medkit-gateway
│   └── fault-manager    — Fault aggregation
│       Apps: medkit-fault-manager
│
└── bridge/              — Legacy diagnostics bridge
    Components
    └── diagnostic-bridge
        Apps: diagnostic-bridge-app, manipulation-monitor

Functions
├── pick-and-place       — Pick objects and place at target positions
├── motion-planning      — Plan collision-free motion trajectories
├── gripper-control      — Open and close the Panda gripper
└── fault-management     — Collect and expose faults via SOVD API
```

## REST API Examples

### Health Check

```bash
curl http://localhost:8080/api/v1/health | jq
```

### Explore Entities

```bash
# Or use the helper script: ./check-entities.sh

curl http://localhost:8080/api/v1/areas | jq
curl http://localhost:8080/api/v1/components | jq
curl http://localhost:8080/api/v1/apps | jq
curl http://localhost:8080/api/v1/functions | jq
```

### Read Joint States

```bash
curl http://localhost:8080/api/v1/apps/joint-state-broadcaster/data/%2Fjoint_states | jq
```

### List MoveIt Configurations

```bash
curl http://localhost:8080/api/v1/apps/move-group/configurations | jq
```

### List Available Operations

```bash
curl http://localhost:8080/api/v1/apps/move-group/operations | jq
```

### View Active Faults

```bash
# Or use the helper script: ./check-faults.sh

curl http://localhost:8080/api/v1/faults | jq
```

### Clear All Faults

```bash
curl -X DELETE http://localhost:8080/api/v1/faults
```

### View Fault Snapshots

When a fault is detected, the fault manager captures environment snapshots (freeze frames) from configured ROS 2 topics. Snapshots are embedded in the fault detail response:

```bash
# Get fault detail with snapshots
curl http://localhost:8080/api/v1/apps/manipulation-monitor/faults/MOTION_PLANNING_FAILED | jq '.environment_data.snapshots'
```

Captured topics (on-demand, 2s timeout):
- `/joint_states` — Current joint positions at fault time
- `/diagnostics` — Active diagnostics messages

> **Note:** Action status topics (`/move_action/_action/status`, `/panda_arm_controller/follow_joint_trajectory/_action/status`) may timeout during snapshot capture since they only publish on state transitions.

### Modify Configurations via REST API

You can read and write ROS 2 node parameters through the gateway:

```bash
# List all parameters for an app
curl http://localhost:8080/api/v1/apps/panda-arm-controller/configurations | jq

# Read a specific parameter
curl http://localhost:8080/api/v1/apps/panda-arm-controller/configurations/gains.panda_joint1.p | jq

# Set a parameter value
curl -X PUT http://localhost:8080/api/v1/apps/panda-arm-controller/configurations/constraints.goal_time \
  -H 'Content-Type: application/json' \
  -d '{"data": {"value": 0.5}}'
```

## Fault Injection Scenarios

### 1. Planning Failure

Blocks the robot's path with a large collision wall.

```bash
./inject-planning-failure.sh
```

| Code | Severity | Description |
|------|----------|-------------|
| `MOTION_PLANNING_FAILED` | ERROR | MoveGroup goal ABORTED — no collision-free path |

### 2. Grasp Failure

Moves the target object far outside the arm's reachable workspace.

> **Note:** This injection only works if the pick-place loop uses the `target_cylinder` collision object as its grasp target. With the default hardcoded joint/cartesian targets, this injection may have no visible effect.

```bash
./inject-grasp-failure.sh
```

| Code | Severity | Description |
|------|----------|-------------|
| `MOTION_PLANNING_FAILED` | ERROR | Cannot plan to unreachable target position |

### 3. Controller Timeout

Sets extremely tight goal time tolerance on the arm controller.

> **Note:** This injection has no effect with fake/mock hardware because the simulated controller reports instant success. Use with Gazebo physics (`--gazebo`) or a physical robot for visible faults.

```bash
./inject-controller-timeout.sh
```

| Code | Severity | Description |
|------|----------|-------------|
| `CONTROLLER_TIMEOUT` | ERROR | Joint trajectory controller timed out |
| `TRAJECTORY_EXECUTION_FAILED` | ERROR | Arm controller ABORTED trajectory |

### 4. Joint Limit Violation

Commands the arm to reach extreme joint positions near/beyond URDF limits.

> **Note:** While the pick-place loop is running, the controller accepts only one goal at a time and may reject external trajectory commands. This injection works best when the pick-place loop is paused.

```bash
./inject-joint-limit.sh
```

| Code | Severity | Description |
|------|----------|-------------|
| `JOINT_LIMIT_APPROACHING` | WARN | Joint within 0.1 rad of limit |
| `JOINT_LIMIT_VIOLATED` | ERROR | Joint position beyond URDF limit |

### 5. Collision Detection

Spawns a surprise obstacle in the robot's active workspace mid-motion.

```bash
./inject-collision.sh
```

| Code | Severity | Description |
|------|----------|-------------|
| `MOTION_PLANNING_FAILED` | ERROR | Cannot find collision-free path around obstacle |

### 6. Restore Normal

Removes all injected objects, restores parameters, and clears faults.

```bash
./restore-normal.sh
```

### Verification

After any injection, verify faults:

```bash
./check-faults.sh
# OR
curl http://localhost:8080/api/v1/faults | jq '.items[] | {fault_code, severity_label, description}'
```

## Web UI

The sovd_web_ui container starts automatically at **http://localhost:3000**.

Connect it to the gateway at `http://localhost:8080` to browse:
- Entity tree (Areas → Components → Apps)
- Real-time joint state data
- Active faults with severity indicators
- Configuration parameters

## Utility Scripts

| Script | Description |
|--------|-------------|
| `move-arm.sh` | **Interactive arm controller** — move to preset positions |
| `check-entities.sh` | Explore the full SOVD entity hierarchy with sample data |
| `check-faults.sh` | View active faults with severity summary |
| `inject-planning-failure.sh` | Block robot path with collision wall |
| `inject-grasp-failure.sh` | Move target object out of reach |
| `inject-controller-timeout.sh` | Set extremely tight goal time tolerance |
| `inject-joint-limit.sh` | Command extreme joint positions |
| `inject-collision.sh` | Spawn surprise obstacle |
| `restore-normal.sh` | Remove all injected faults and restore defaults |

## Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| RViz window doesn't appear | X11 not set up | Run `xhost +local:docker` or use `--headless` |
| "Package not found" error | Build failed | Rebuild with `./run-demo.sh --no-cache` |
| No faults appearing | Monitor not connected | Check `ros2 node list` includes `manipulation_monitor` |
| Docker build fails | Apt package missing | Check if MoveIt 2 Jazzy packages are available |
| "MoveGroup not available" | Slow startup | Wait 60-90 seconds after container starts |
| Controller not loading | Missing config | Verify `moveit_controllers.yaml` is correct |
| Joint states empty | Controllers not loaded | Check `ros2 control list_controllers` inside container |
| `ros2` CLI hangs in `docker exec` | DDS discovery across container boundaries | Use gateway REST API instead of `ros2` CLI for parameter/service operations |
| Injection script has no output | DDS multicast not reachable from host | Run injection scripts inside the demo container or use REST API equivalents |

## Comparison with Other Demos

| Feature | Sensor Diagnostics | TurtleBot3 + Nav2 | **MoveIt Pick-and-Place** |
|---------|-------------------|-------------------|---------------------------|
| Robot | Simulated sensors | TurtleBot3 Burger | Panda 7-DOF arm |
| Simulation | None (pure ROS 2) | Gazebo Harmonic | Fake HW or Gazebo Harmonic |
| Task | Sensor monitoring | Autonomous navigation | Pick-and-place manipulation |
| Fault types | Sensor drift, noise | Nav failures, localization | Planning, controller, joint limits |
| Entity complexity | Simple (flat) | Medium (3 areas) | High (4 areas, 7 components) |
| SOVD manifest | No | Yes (hybrid) | Yes (hybrid) |
| Docker image | ~2 GB | ~4 GB | ~7 GB |
| GPU recommended | No | Optional | Optional |

## Technical Details

### Monitored Topics

| Topic | What it tells us | Fault codes |
|-------|------------------|-------------|
| `/move_action/_action/status` | Planning success/failure | `MOTION_PLANNING_FAILED` |
| `/panda_arm_controller/follow_joint_trajectory/_action/status` | Trajectory execution | `TRAJECTORY_EXECUTION_FAILED`, `CONTROLLER_TIMEOUT` |
| `/joint_states` | Current joint positions | `JOINT_LIMIT_APPROACHING`, `JOINT_LIMIT_VIOLATED` |

### Fault Codes Reference

| Code | Severity | Trigger |
|------|----------|---------|
| `MOTION_PLANNING_FAILED` | ERROR | MoveGroup goal ABORTED |
| `TRAJECTORY_EXECUTION_FAILED` | ERROR | Controller action ABORTED |
| `CONTROLLER_TIMEOUT` | ERROR | Controller action ABORTED (timeout) |
| `JOINT_LIMIT_APPROACHING` | WARN | Joint within warn margin of URDF limit |
| `JOINT_LIMIT_VIOLATED` | ERROR | Joint position beyond URDF limit |

### Docker Image Contents

- ROS 2 Jazzy Desktop (Ubuntu 24.04)
- MoveIt 2 + OMPL planner
- Panda URDF + MoveIt config
- Gazebo Harmonic + gz_ros2_control (for `--gazebo` mode)
- ros2_control with mock hardware (for default mode)
- ros2_medkit stack (gateway, fault_manager, diagnostic_bridge)
- Demo package (launch, config, scripts)

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](../../LICENSE) file for details.
