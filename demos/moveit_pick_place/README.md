# MoveIt 2 Pick-and-Place Integration Demo

A comprehensive integration demo combining a **Panda 7-DOF robot arm** with **MoveIt 2** motion planning and **ros2_medkit** SOVD-compliant diagnostics. The robot performs continuous pick-and-place cycles in a **Gazebo Harmonic factory scene** while a manipulation monitor detects faults - planning failures, collisions - and reports them through the SOVD REST API with environment snapshots.

## Status

✅ **Demo Ready** - Docker-based deployment with MoveIt 2, Gazebo Harmonic physics simulation, factory environment, and full ros2_medkit stack.

## Overview

This demo demonstrates:

- **MoveIt 2 motion planning** with the Panda 7-DOF arm and gripper
- **Gazebo Harmonic simulation** with a realistic factory scene (conveyor belt, work table, storage, lighting)
- **Continuous pick-and-place** loop as a realistic manipulation workload
- **Manipulation fault monitoring** (planning failures, collision detection)
- **Fault snapshots** - environment state captured at fault time (joint states, diagnostics)
- **SOVD-compliant REST API** with Areas → Components → Apps → Functions hierarchy
- **Manifest-based entity discovery** (hybrid mode with runtime enrichment)
- **2 fault injection scenarios** with visible Gazebo models and one-click scripts
- **Web UI** for visual entity browsing and fault monitoring

## Prerequisites

- Docker and docker-compose
- `curl` and `jq` installed on the host (required for host-side scripts)
- X11 display server (for Gazebo GUI) or `--headless` mode
- (Optional) NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) - recommended for smooth Gazebo rendering
- ~7 GB disk space for Docker image

## Quick Start

### 1. Start the Demo

```bash
cd demos/moveit_pick_place
./run-demo.sh
```

That's it! The script will:
1. Build the Docker image (first run: ~15-20 min, ~7 GB)
2. Set up X11 forwarding for Gazebo GUI
3. Launch Panda robot in factory world + MoveIt 2 + ros2_medkit gateway
4. Launch ros2_medkit_web_ui at http://localhost:3000

**REST API:** http://localhost:8080/api/v1/
**Web UI:** http://localhost:3000/

### 2. Available Options

```bash
./run-demo.sh                  # Default: Gazebo simulation, daemon mode
./run-demo.sh --nvidia         # GPU acceleration (recommended)
./run-demo.sh --headless       # No GUI (CI/server)
./run-demo.sh --attached       # Foreground with logs
./run-demo.sh --no-cache       # Rebuild without cache
./run-demo.sh --update         # Pull latest images first
```

The demo always uses **Gazebo Harmonic** physics simulation with `gz_ros2_control`. The factory scene includes a work table, conveyor belt, storage bin, and industrial lighting. Use `--nvidia` for GPU-accelerated rendering.

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
│  │ Gazebo   │  │  ros2_control │  │     MoveIt 2             │  │
│  │ Harmonic │──│  Controllers  │──│  move_group (OMPL)       │  │
│  │ (physics)│  │  arm+gripper  │  │                          │  │
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
│  │         │ monitors:              │ stores faults +         │   │
│  │         │ • /move_action status  │ captures snapshots     │   │
│  │         │ • /controller status   ▼                        │   │
│  │         │ • /joint_states    gateway_node ──► REST API    │   │
│  └─────────┴────────────────────────┬───────────────────────┘   │
│                                     │ :8080                      │
└─────────────────────────────────────┼────────────────────────────┘
                                      │
                        ┌─────────────┼─────────────┐
                        │             │             │
                   ros2_medkit_web_ui    curl/httpie    MCP Server
                    :3000                        (LLM tools)
```

## Entity Hierarchy (SOVD)

```
Areas
├── manipulation/        - Robot arm and gripper hardware
│   Components
│   ├── panda-arm        - 7-DOF Franka Emika Panda
│   │   Apps: joint-state-broadcaster, panda-arm-controller, robot-state-publisher
│   └── panda-gripper    - 2-finger parallel gripper
│       Apps: panda-hand-controller
│
├── planning/            - MoveIt 2 motion planning stack
│   Components
│   ├── moveit-planning  - OMPL planning pipeline
│   │   Apps: move-group
│   └── pick-place-loop - Pick-and-place demo node
│       Apps: pick-place-node
│
├── diagnostics/         - ros2_medkit gateway and fault management
│   Components
│   ├── gateway          - REST API
│   │   Apps: medkit-gateway
│   └── fault-manager    - Fault aggregation
│       Apps: medkit-fault-manager
│
└── bridge/              - Legacy diagnostics bridge
    Components
    └── diagnostic-bridge
        Apps: diagnostic-bridge-app, manipulation-monitor

Functions
├── pick-and-place       - Pick objects and place at target positions
├── motion-planning      - Plan collision-free motion trajectories
├── gripper-control      - Open and close the Panda gripper
└── fault-management     - Collect and expose faults via SOVD API
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

Captured topics (background capture, always available):
- `/joint_states` - Current joint positions at fault time
- `/diagnostics` - Active diagnostics messages

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
  -d '{"value": 0.5}'
```

## Scripts API

The gateway exposes a Scripts API for the `moveit-planning` component. All fault injection and diagnostic scripts are available via REST without `docker exec`.

### Available Scripts

| Script ID | Name | Description |
|-----------|------|-------------|
| `inject-collision` | Inject Collision | Spawn a surprise obstacle in the robot workspace (Gazebo + MoveIt planning scene) |
| `inject-planning-failure` | Inject Planning Failure | Add collision wall blocking the pick-place path (Gazebo + MoveIt planning scene) |
| `restore-normal` | Restore Normal | Remove all injected obstacles and clear faults |
| `arm-self-test` | Arm Self-Test | Check joint states via REST API, verify values are reasonable |
| `planning-benchmark` | Planning Benchmark | Verify MoveIt planning is functional by checking key nodes and operations |

### List Available Scripts

```bash
curl http://localhost:8080/api/v1/components/moveit-planning/scripts | jq
```

### Execute a Script via REST

```bash
# Start execution
curl -X POST http://localhost:8080/api/v1/components/moveit-planning/scripts/inject-collision/executions \
  -H "Content-Type: application/json" \
  -d '{"execution_type": "now"}' | jq

# Poll status (use execution ID from above response)
curl http://localhost:8080/api/v1/components/moveit-planning/scripts/inject-collision/executions/<exec_id> | jq
```

### Override Gateway URL

```bash
# Point scripts at a non-default gateway
GATEWAY_URL=http://192.168.1.10:8080 ./inject-collision.sh
```

The host-side wrapper scripts (`./inject-collision.sh`, etc.) call the Scripts API automatically - no `docker exec` needed. Prerequisites: `curl` and `jq` must be installed on the host.

## Triggers (Condition-Based Alerts)

The gateway supports condition-based triggers that fire when specific events occur, delivering notifications via Server-Sent Events (SSE). This demo creates a fault-monitoring trigger that alerts on planning failure faults reported by the manipulation monitor.

### Setup

```bash
# Terminal 1: Start the demo
./run-demo.sh

# Terminal 2: Create the fault trigger
./setup-triggers.sh

# Terminal 3: Watch for trigger events (blocking - Ctrl+C to stop)
./watch-triggers.sh

# Terminal 2: Inject a fault - the trigger fires in Terminal 3!
./inject-planning-failure.sh
```

### How It Works

1. `setup-triggers.sh` creates a trigger via `POST /api/v1/apps/manipulation_monitor/triggers`:
   - **Resource:** `/api/v1/apps/manipulation_monitor/faults` (watches fault collection)
   - **Condition:** `OnChange` (fires on any new or updated fault)
   - **Multishot:** `true` (fires repeatedly, not just once)
   - **Lifetime:** 3600 seconds (auto-expires after 1 hour)
2. `watch-triggers.sh` connects to the SSE event stream at the trigger's `event_source` URL
3. When a fault is injected and detected by the gateway, the trigger fires and an SSE event is delivered

### Manual API Usage

```bash
# Create a trigger
curl -X POST http://localhost:8080/api/v1/apps/manipulation_monitor/triggers \
  -H "Content-Type: application/json" \
  -d '{
    "resource": "/api/v1/apps/manipulation_monitor/faults",
    "trigger_condition": {"condition_type": "OnChange"},
    "multishot": true,
    "lifetime": 3600
  }' | jq

# List triggers
curl http://localhost:8080/api/v1/apps/manipulation_monitor/triggers | jq

# Watch events (replace TRIGGER_ID)
curl -N http://localhost:8080/api/v1/apps/manipulation_monitor/triggers/TRIGGER_ID/events

# Delete a trigger
curl -X DELETE http://localhost:8080/api/v1/apps/manipulation_monitor/triggers/TRIGGER_ID
```

## Fault Injection Scenarios

The fault injection scripts run inside the container via the Scripts API. The host-side `./inject-*.sh` and `./restore-normal.sh` wrappers call the gateway REST endpoint - no `docker exec` required.

### 1. Planning Failure

Blocks the robot's path with a large collision wall (visible as orange wall in Gazebo).

```bash
./inject-planning-failure.sh
```

| Code | Severity | Description |
|------|----------|-------------|
| `MOTION_PLANNING_FAILED` | ERROR | MoveGroup goal ABORTED - no collision-free path |

### 2. Collision Detection

Spawns a surprise obstacle in the robot's active workspace (visible as red sphere in Gazebo).

```bash
./inject-collision.sh
```

| Code | Severity | Description |
|------|----------|-------------|
| `MOTION_PLANNING_FAILED` | ERROR | Cannot find collision-free path around obstacle |

### 3. Restore Normal

Removes all injected objects (from both Gazebo and MoveIt planning scene) and clears faults.

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

The ros2_medkit_web_ui container starts automatically at **http://localhost:3000**.

Connect it to the gateway at `http://localhost:8080` to browse:
- Entity tree (Areas → Components → Apps)
- Real-time joint state data
- Active faults with severity indicators
- Configuration parameters

## Utility Scripts

### Host-side (run from your machine)

| Script | Description |
|--------|-------------|
| `run-demo.sh` | **Start the demo** - build and launch the Docker container |
| `stop-demo.sh` | Stop demo containers |
| `move-arm.sh` | **Interactive arm controller** - move to preset positions |
| `check-entities.sh` | Explore the full SOVD entity hierarchy with sample data |
| `check-faults.sh` | View active faults with severity summary |
| `inject-planning-failure.sh` | Scripts API wrapper - inject planning failure |
| `inject-collision.sh` | Scripts API wrapper - inject collision obstacle |
| `restore-normal.sh` | Scripts API wrapper - restore normal operation |
| `arm-self-test.sh` | Scripts API wrapper - run arm self-test |
| `planning-benchmark.sh` | Scripts API wrapper - run planning benchmark |
| `setup-triggers.sh` | Create OnChange fault trigger |
| `watch-triggers.sh` | Watch trigger events via SSE stream |

Scripts API wrappers require `curl` and `jq` on the host and call the gateway REST endpoint directly - no `docker exec` needed.

### In-container scripts (auto-discovery via Scripts API)

Container scripts are stored under `/var/lib/ros2_medkit/scripts/moveit-planning/` and exposed via the gateway Scripts API.

| Script ID | Description |
|-----------|-------------|
| `inject-collision` | Spawn visible sphere + MoveIt collision object |
| `inject-planning-failure` | Spawn visible wall + MoveIt collision object |
| `restore-normal` | Remove Gazebo models + MoveIt objects, clear faults |
| `arm-self-test` | Check joint states via REST API |
| `planning-benchmark` | Verify MoveIt planning is functional |

### ROS 2 runtime scripts (baked into colcon install)

| Script | Description |
|--------|-------------|
| `manipulation_monitor.py` | ROS 2 node: monitors topics and reports faults |
| `pick_place_loop.py` | ROS 2 node: continuous pick-and-place cycle |

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

## Comparison with Other Demos

| Feature | Sensor Diagnostics | TurtleBot3 + Nav2 | **MoveIt Pick-and-Place** |
|---------|-------------------|-------------------|---------------------------|
| Robot | Simulated sensors | TurtleBot3 Burger | Panda 7-DOF arm |
| Simulation | None (pure ROS 2) | Gazebo Harmonic | Gazebo Harmonic |
| Task | Sensor monitoring | Autonomous navigation | Pick-and-place manipulation |
| Fault types | Sensor drift, noise | Nav failures, localization | Planning failures, collisions |
| Entity complexity | Simple (flat) | Medium (3 areas) | High (4 areas, 7 components) |
| SOVD manifest | No | Yes (hybrid) | Yes (hybrid) |
| Docker image | ~2 GB | ~4 GB | ~7 GB |
| GPU recommended | No | Optional | Recommended |

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
| `MOTION_PLANNING_FAILED` | ERROR | MoveGroup goal ABORTED (collision wall or obstacle) |
| `TRAJECTORY_EXECUTION_FAILED` | ERROR | Controller action ABORTED |
| `JOINT_LIMIT_APPROACHING` | WARN | Joint within warn margin of URDF limit |
| `JOINT_LIMIT_VIOLATED` | ERROR | Joint position beyond URDF limit |

### Docker Image Contents

- ROS 2 Jazzy Desktop (Ubuntu 24.04)
- MoveIt 2 + OMPL planner
- Panda URDF + MoveIt config
- Gazebo Harmonic + gz_ros2_control
- Factory world scene (SDF)
- ros2_medkit stack (gateway, fault_manager, diagnostic_bridge)
- Demo package (launch, config, scripts)

## License

This project is licensed under the Apache License 2.0. See the [LICENSE](../../LICENSE) file for details.
