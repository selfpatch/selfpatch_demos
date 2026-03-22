# TurtleBot3 Integration Demo with Nav2 Navigation

This demo shows how to integrate ros2_medkit with TurtleBot3 and Nav2 navigation stack
to provide SOVD-compliant diagnostics, fault management, and control for a mobile robot system via REST API.

## Status

✅ **Demo Ready** - Full navigation demo with Web UI and fault management

## Overview

This demo demonstrates:

- Launching TurtleBot3 simulation in Gazebo with turtlebot3_world
- Running Nav2 navigation stack (AMCL, planner, controller)
- Running ros2_medkit gateway with **manifest-based discovery**
- Fault management via **diagnostic_bridge** (legacy /diagnostics support)
- **Rosbag snapshot capture** when faults are confirmed (MCAP format)
- Querying robot data via **REST API**
- Entity hierarchy: Areas → Components → Apps → Functions
- Controlling the robot via ros2_medkit_web_ui

## Prerequisites

- Docker and docker-compose
- X11 display server (Linux with GUI, or XQuartz on macOS)
- (Optional) NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- `curl` and `jq` (required for host-side scripts)

## Quick Start

### 1. Start the ROS2 Backend

```bash
cd demos/turtlebot3_integration
./run-demo.sh
```

That's it! The script will:

1. Build the Docker images (first run takes ~5-10 min, downloads ~4GB)
2. Setup X11 forwarding for Gazebo GUI
3. Launch TurtleBot3 simulation + Nav2 + ros2_medkit gateway in **daemon mode** (background)
4. Launch ros2_medkit_web_ui at <http://localhost:3000>

**Note:** By default, the demo runs in **daemon mode** with **Gazebo GUI** enabled. This allows you to interact with ROS 2 while the demo is running.

### Available Options

```bash
./run-demo.sh              # Daemon mode with GUI (default)
./run-demo.sh --attached   # Foreground mode with live logs
./run-demo.sh --headless   # Daemon mode without GUI (headless)
./run-demo.sh --nvidia     # Use NVIDIA GPU acceleration
./run-demo.sh --update     # Pull latest images before running
./run-demo.sh --no-cache   # Rebuild without cache
```

### Viewing Logs and Interacting with ROS 2

Since the demo runs in daemon mode by default, you can:

```bash
# View live logs
docker compose --profile cpu logs -f

# Enter the container to run ROS 2 commands
docker exec -it turtlebot3_medkit_demo bash

# Inside the container:
ros2 node list
ros2 topic list
ros2 topic echo /odom
```

### Stopping the Demo

```bash
./stop-demo.sh              # Stop containers
./stop-demo.sh --images     # Stop and remove images
```

**Note:** Fault data and rosbag recordings are ephemeral — they are stored
inside the container and cleared on restart.

### 2. Access the Web UI

The Web UI is automatically started by docker-compose and available at <http://localhost:3000>.

Connect to the gateway using `http://localhost:8080/api/v1` in the connection dialog.

### With NVIDIA GPU

For hardware-accelerated Gazebo rendering with NVIDIA GPU:

```bash
./run-demo.sh --nvidia
```

**Requirements:**

- NVIDIA GPU with recent drivers
- [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) installed

You can also use Docker Compose directly:

```bash
# CPU version (default profile)
docker compose --profile cpu up -d

# NVIDIA version
docker compose --profile nvidia up -d

# View logs
docker compose --profile cpu logs -f
docker compose --profile nvidia logs -f
```

## Controlling the Robot

### Via Web UI

1. Connect to the gateway in ros2_medkit_web_ui
2. Find entity with `/cmd_vel` data
3. Enter velocity command JSON (or use form with fields from schema):

   ```json
   {"linear": {"x": 0.2}, "angular": {"z": 0.0}}
   ```

4. Click "Send" - the robot will move!

### Via Command Line

```bash
# Send velocity command using Apps data endpoint (moves robot forward)
curl -X PUT http://localhost:8080/api/v1/apps/turtlebot3-node/data/cmd_vel \
  -H "Content-Type: application/json" \
  -d '{"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}'

# Stop the robot
curl -X PUT http://localhost:8080/api/v1/apps/turtlebot3-node/data/cmd_vel \
  -H "Content-Type: application/json" \
  -d '{"linear": {"x": 0.0}, "angular": {"z": 0.0}}'
```

### Via ROS2 CLI (inside container)

First, enter the running container:

```bash
# Enter the container
docker exec -it turtlebot3_medkit_demo bash

# Inside the container:
# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"

# Manual teleop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Exit container
exit
```

## REST API Endpoints

### Discovery (SOVD Entity Hierarchy)

```bash
# Check gateway health
curl http://localhost:8080/api/v1/health

# List discovered areas (namespace groupings)
curl http://localhost:8080/api/v1/areas | jq '.items[] | {id, name}'

# List all components (hardware/logical units)
curl http://localhost:8080/api/v1/components | jq '.items[] | {id, name, area}'

# List all apps (ROS 2 nodes)
curl http://localhost:8080/api/v1/apps | jq '.items[] | {id, name, namespace}'

# Get specific app details
curl http://localhost:8080/api/v1/apps/amcl | jq
```

### Data Access (via Apps)

```bash
# Get LiDAR scan data
curl http://localhost:8080/api/v1/apps/turtlebot3-node/data/scan | jq '{
  angle_min: .angle_min,
  angle_max: .angle_max,
  sample_ranges: .ranges[:5]
}'

# Get odometry data
curl http://localhost:8080/api/v1/apps/turtlebot3-node/data/odom | jq '{
  position: .pose.pose.position,
  orientation: .pose.pose.orientation
}'

# List all data topics for an app
curl http://localhost:8080/api/v1/apps/turtlebot3-node/data | jq
```

### Fault Management

```bash
# List all active faults
curl http://localhost:8080/api/v1/faults | jq

# Get faults for a specific area
curl http://localhost:8080/api/v1/areas/robot/faults | jq

# Get fault details with environment data (includes snapshots)
curl http://localhost:8080/api/v1/faults/NAVIGATION_GOAL_ABORTED | jq

# Clear a specific fault
curl -X DELETE http://localhost:8080/api/v1/apps/diagnostic-bridge/faults/TURTLEBOT3_NODE
```

### Rosbag Snapshots (Bulk Data)

When a fault is confirmed, the FaultManager automatically captures:
- **Freeze frame snapshots**: Latest messages from key topics (odometry, pose, scan)
- **Rosbag recording**: 10 seconds before + 2 seconds after fault confirmation

```bash
# List bulk-data categories for an entity
curl http://localhost:8080/api/v1/apps/{entity_id}/bulk-data | jq

# List rosbag files available for download
curl http://localhost:8080/api/v1/apps/{entity_id}/bulk-data/rosbags | jq

# Download a rosbag file (returns MCAP format)
curl -O http://localhost:8080/api/v1/apps/{entity_id}/bulk-data/rosbags/{fault_code}

# Get fault detail with snapshots (freeze frames)
curl http://localhost:8080/api/v1/apps/{entity_id}/faults/{fault_code} | jq
```

**Recorded Topics:**
- `/odom`, `/amcl_pose`, `/scan` - Robot state
- `/cmd_vel` - Velocity commands
- `/tf`, `/tf_static` - Transforms
- `/navigate_to_pose/_action/status`, `/navigate_to_pose/_action/feedback` - Navigation state
- `/local_costmap/costmap`, `/global_costmap/costmap` - Costmaps
- `/plan` - Navigation plan
- `/diagnostics` - System diagnostics

### Operations (Service Calls)

```bash
# List available operations for an app
curl http://localhost:8080/api/v1/apps/amcl/operations | jq

# Execute an operation (service call)
curl -X POST http://localhost:8080/api/v1/apps/amcl/operations/reinitialize_global_localization/executions \
  -H "Content-Type: application/json" \
  -d '{}'
```

### Configurations (Parameters)

```bash
# List node parameters
curl http://localhost:8080/api/v1/apps/amcl/configurations | jq

# Get a specific parameter
curl http://localhost:8080/api/v1/apps/amcl/configurations/max_particles | jq

# Update a parameter
curl -X PUT http://localhost:8080/api/v1/apps/amcl/configurations/max_particles \
  -H "Content-Type: application/json" \
  -d '{"value": 3000}'
```

## What You'll See

When TurtleBot3 simulation starts with Nav2, ros2_medkit will discover nodes organized into the **SOVD entity hierarchy** defined by the manifest:

### Entity Hierarchy

```
TurtleBot3 Demo (manifest-based discovery)
├── Areas (namespace groupings)
│   ├── robot         → TurtleBot3 hardware
│   ├── navigation    → Nav2 stack
│   ├── diagnostics   → ros2_medkit gateway
│   └── bridge        → Diagnostic bridge
├── Components (hardware/logical units)
│   ├── turtlebot3-base  → Robot platform (area: robot)
│   ├── lidar-sensor     → LiDAR scanner (area: robot)
│   ├── nav2-stack       → Navigation (area: navigation)
│   ├── gateway          → REST API (area: diagnostics)
│   ├── fault-manager    → Fault aggregation (area: diagnostics)
│   └── diagnostic-bridge-unit → Legacy support (area: bridge)
├── Apps (ROS 2 nodes)
│   ├── turtlebot3-node      → component: turtlebot3-base
│   ├── robot-state-publisher → component: turtlebot3-base
│   ├── amcl                  → component: nav2-stack
│   ├── bt-navigator          → component: nav2-stack
│   ├── controller-server     → component: nav2-stack
│   ├── planner-server        → component: nav2-stack
│   ├── medkit-gateway        → component: gateway
│   ├── medkit-fault-manager  → component: fault-manager
│   ├── diagnostic-bridge     → component: diagnostic-bridge-unit
│   └── anomaly-detector      → component: diagnostic-bridge-unit
└── Functions (high-level capabilities)
    ├── autonomous-navigation → hosted by: amcl, bt-navigator, ...
    ├── robot-control         → hosted by: turtlebot3-node, velocity-smoother
    └── fault-management      → hosted by: gateway, fault-manager, bridge, anomaly-detector
```

### Fault Reporting

This demo uses two fault reporting paths:

1. **Direct Fault Reporting** via `anomaly_detector`:
   - Monitors navigation goal status, AMCL covariance, and robot progress
   - Reports faults directly to FaultManager via `/fault_manager/report_fault` service

2. **Legacy Path** via `diagnostic_bridge`:
   - Subscribes to `/diagnostics` topic (DiagnosticArray)
   - Converts diagnostics to fault reports

```
┌─────────────────────────────────────────────────────────────────────┐
│                     Fault Reporting Paths                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  anomaly_detector ─────┬──── /fault_manager/report_fault ──────┐   │
│    (monitors:          │            (direct service call)       │   │
│     - nav goal status) │                                        │   │
│     - AMCL covariance) │                                        ▼   │
│     - robot progress)  │                              ┌──────────────┐
│                        │                              │ FaultManager │
│                        │                              └──────┬───────┘
│  Nav2/TurtleBot3 ──────┼──── /diagnostics topic ──────┐     │        │
│    (DiagnosticArray)   │                              ▼     │        │
│                        │                   diagnostic_bridge │        │
│                        └───────────────────────────────┘     │        │
│                                                              ▼        │
│                                           GET /api/v1/faults         │
└─────────────────────────────────────────────────────────────────────┘
```

| Source | Fault Reporter | Example Faults |
|--------|----------------|----------------|
| Navigation Goals | anomaly_detector | `NAVIGATION_GOAL_ABORTED`, `NAVIGATION_GOAL_CANCELED` |
| Localization | anomaly_detector | `LOCALIZATION_UNCERTAINTY` |
| Robot Progress | anomaly_detector | `NAVIGATION_NO_PROGRESS` |
| AMCL | diagnostic_bridge | Localization degraded |
| Nav2 Controller | diagnostic_bridge | Path following errors |
| TurtleBot3 | diagnostic_bridge | Motor/sensor issues |

## Scripts API

The inject and restore scripts run inside the container and are callable via the gateway REST API. This lets you trigger fault scenarios programmatically or from the web UI.

> **Host prerequisites:** The host-side scripts require `curl` and `jq`.

### List Available Scripts

```bash
curl http://localhost:8080/api/v1/components/nav2-stack/scripts | jq
```

### Execute a Script

```bash
curl -X POST http://localhost:8080/api/v1/components/nav2-stack/scripts/inject-nav-failure/executions \
  -H "Content-Type: application/json" \
  -d '{"execution_type":"now"}' | jq
```

### Check Execution Status

```bash
curl http://localhost:8080/api/v1/components/nav2-stack/scripts/inject-nav-failure/executions/<exec_id> | jq
```

### Override Gateway URL

```bash
# Point scripts at a non-default gateway
GATEWAY_URL=http://192.168.1.10:8080 ./inject-nav-failure.sh
```

### Available Scripts

| Script | Description |
|--------|-------------|
| `nav-health-check` | Check health of Nav2 stack |
| `reset-navigation` | Cancel goals and reset AMCL |
| `inject-localization-failure` | Inject AMCL localization failure |
| `inject-nav-failure` | Inject navigation failure (unreachable goal) |
| `restore-normal` | Reset parameters and clear faults |

## Triggers (Condition-Based Alerts)

The gateway supports condition-based triggers that fire when specific events occur, delivering notifications via Server-Sent Events (SSE). This demo creates a fault-monitoring trigger that alerts on navigation failure faults on the nav2-stack component.

### Setup

```bash
# Terminal 1: Start the demo
./run-demo.sh

# Terminal 2: Create the fault trigger
./setup-triggers.sh

# Terminal 3: Watch for trigger events (blocking - Ctrl+C to stop)
./watch-triggers.sh

# Terminal 2: Inject a fault - the trigger fires in Terminal 3!
./inject-nav-failure.sh
```

### How It Works

1. `setup-triggers.sh` creates a trigger via `POST /api/v1/components/nav2-stack/triggers`:
   - **Resource:** `/api/v1/components/nav2-stack/faults` (watches fault collection)
   - **Condition:** `OnChange` (fires on any new or updated fault)
   - **Multishot:** `true` (fires repeatedly, not just once)
   - **Lifetime:** 3600 seconds (auto-expires after 1 hour)
2. `watch-triggers.sh` connects to the SSE event stream at the trigger's `event_source` URL
3. When a fault is injected and detected by the gateway, the trigger fires and an SSE event is delivered

### Manual API Usage

```bash
# Create a trigger
curl -X POST http://localhost:8080/api/v1/components/nav2-stack/triggers \
  -H "Content-Type: application/json" \
  -d '{
    "resource": "/api/v1/components/nav2-stack/faults",
    "trigger_condition": {"condition_type": "OnChange"},
    "multishot": true,
    "lifetime": 3600
  }' | jq

# List triggers
curl http://localhost:8080/api/v1/components/nav2-stack/triggers | jq

# Watch events (replace TRIGGER_ID)
curl -N http://localhost:8080/api/v1/components/nav2-stack/triggers/TRIGGER_ID/events

# Delete a trigger
curl -X DELETE http://localhost:8080/api/v1/components/nav2-stack/triggers/TRIGGER_ID
```

## Fault Injection Scenarios

This demo includes scripts to inject various fault conditions for testing fault management.
Faults are detected by `anomaly_detector` and reported directly to FaultManager.

### Available Fault Scenarios

| Script | Fault Type | Description | Expected Faults |
|--------|-----------|-------------|-----------------|
| `inject-nav-failure.sh` | Navigation | Send goal to unreachable location | `NAVIGATION_GOAL_ABORTED` |
| `inject-localization-failure.sh` | Localization | Reset AMCL with high uncertainty | `LOCALIZATION_UNCERTAINTY` |
| `restore-normal.sh` | Recovery | Restore defaults and clear faults | - |

### Fault Injection Examples

#### 1. Navigation Failure

```bash
# Send robot to unreachable goal (outside map bounds)
./inject-nav-failure.sh

# Check resulting faults
curl http://localhost:8080/api/v1/faults | jq '.items[] | {fault_code, severity_label, description}'
```

#### 2. Localization Failure

```bash
# Reinitialize AMCL global localization (causes high uncertainty)
./inject-localization-failure.sh

# Watch for LOCALIZATION_UNCERTAINTY fault
curl http://localhost:8080/api/v1/faults | jq
```

#### Restore Normal Operation

```bash
# Clear all faults and restore default parameters
./restore-normal.sh
```

### Fault Monitoring via API

```bash
# List all active faults
curl http://localhost:8080/api/v1/faults | jq

# Get faults for navigation area
curl http://localhost:8080/api/v1/areas/navigation/faults | jq

# Clear specific fault
curl -X DELETE http://localhost:8080/api/v1/apps/{entity_id}/faults/{fault_code}

# Clear all faults
curl -X DELETE http://localhost:8080/api/v1/faults
```

## Architecture

```text
┌──────────────────────────────────────────────────────────────────────┐
│                       Docker Container                               │
│  ┌───────────────────────────────────────────────────────────────┐   │
│  │                   Gazebo Simulation                           │   │
│  │  ┌─────────────┐  ┌──────────────┐  ┌────────┐ ┌───────────┐  │   │
│  │  │ TurtleBot3  │  │ robot_state  │  │ LIDAR  │ │   Nav2    │  │   │
│  │  │    Node     │  │  publisher   │  │ sensor │ │ (AMCL,    │  │   │
│  │  └──────┬──────┘  └──────┬───────┘  └───┬────┘ │ Planner,  │  │   │
│  │         │                │              │      │ Controller│  │   │
│  │         └────────────────┼──────────────┘      └─────┬─────┘  │   │
│  │                    ROS 2 Topics ◄────────────────────┘        │   │
│  └──────────────────────────┼────────────────────────────────────┘   │
│                             │                                        │
│           ┌─────────────────┼─────────────────┐                      │
│           │                 │                 │                      │
│           ▼                 ▼                 ▼                      │
│  ┌────────────────┐  ┌─────────────┐  ┌──────────────────┐           │
│  │ /diagnostics   │  │ fault_      │  │ ros2_medkit      │           │
│  │ topic          │  │ manager     │  │ Gateway          │           │
│  └───────┬────────┘  └──────▲──────┘  │ (REST :8080)     │           │
│          │                  │         └────────┬─────────┘           │
│          ▼                  │                  │                     │
│  ┌────────────────┐         │                  │                     │
│  │ diagnostic_    ├─────────┘                  │                     │
│  │ bridge         │                            │                     │
│  └────────────────┘                            │                     │
└────────────────────────────────────────────────┼─────────────────────┘
                                                 │
                                          HTTP REST API
                                                 │
         ┌───────────────────────────────────────┼────────────────────┐
         ▼                    ▼                  ▼                    ▼
   ros2_medkit_web_ui          curl/browser      External tools        MCP Server
  (localhost:3000)                                          (ros2_medkit_mcp)
```

## File Structure

```text
demos/turtlebot3_integration/
├── Dockerfile                  # ROS 2 Jazzy + TurtleBot3 + Nav2 + ros2_medkit
├── docker-compose.yml          # Docker Compose (CPU & GPU via profiles)
├── run-demo.sh                 # One-click demo launcher
├── stop-demo.sh                # Stop and cleanup demo
├── send-nav-goal.sh            # Send navigation goal via SOVD API
├── check-entities.sh           # Explore SOVD entity hierarchy
├── check-faults.sh             # View active faults
├── inject-nav-failure.sh       # Inject navigation failure scenario
├── inject-localization-failure.sh  # Inject AMCL localization issues
├── restore-normal.sh           # Restore normal operation
├── config/
│   ├── medkit_params.yaml           # ros2_medkit gateway config
│   ├── turtlebot3_manifest.yaml     # SOVD manifest (entity hierarchy)
│   ├── nav2_params.yaml             # Nav2 navigation parameters
│   └── turtlebot3_world.yaml        # Map configuration
├── container_scripts/
│   └── nav2-stack/              # Scripts API auto-discovery layout
│       ├── nav-health-check/
│       ├── reset-navigation/
│       ├── inject-nav-failure/
│       ├── inject-localization-failure/
│       └── restore-normal/
├── launch/
│   └── demo.launch.py          # ROS 2 launch file
└── scripts/
    └── anomaly_detector.py     # Navigation anomaly detector node
```

## Scripts

| Script | Description |
|--------|-------------|
| `run-demo.sh` | Start the full demo (Docker) |
| `stop-demo.sh` | Stop containers and cleanup |
| `send-nav-goal.sh [x] [y] [yaw]` | Send navigation goal via SOVD API |
| `check-entities.sh` | Explore SOVD entity hierarchy |
| `check-faults.sh` | View active faults from gateway |
| `nav-health-check.sh` | Check Nav2 stack health |
| `reset-navigation.sh` | Cancel goals and reset AMCL |
| `inject-nav-failure.sh` | Inject navigation failure (unreachable goal) |
| `inject-localization-failure.sh` | Inject localization failure (AMCL reset) |
| `restore-normal.sh` | Restore normal operation and clear faults |
| `setup-triggers.sh` | Create OnChange fault trigger |
| `watch-triggers.sh` | Watch trigger events via SSE stream |

> **Note:** The inject, restore, and diagnostic scripts are also available via the [Scripts API](#scripts-api) - callable as REST endpoints without requiring the host-side scripts.

## Manual Setup (Alternative)

If you prefer not to use Docker:

1. Install ROS 2 Jazzy on Ubuntu 24.04
2. Install TurtleBot3: `sudo apt install ros-jazzy-turtlebot3-gazebo`
3. Install Nav2: `sudo apt install ros-jazzy-nav2-bringup`
4. Build [ros2_medkit](https://github.com/selfpatch/ros2_medkit) from source
5. Set environment:

   ```bash
   export TURTLEBOT3_MODEL=burger
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
   ```

6. Run: `ros2 launch launch/demo.launch.py`

## Troubleshooting

### Gazebo window doesn't appear

- Ensure X11 forwarding is set up: `xhost +local:docker`
- Check DISPLAY environment variable

### Nav2 doesn't start properly

- Wait for AMCL to localize (give an initial pose in RViz or via CLI)
- Check lifecycle states with `ros2 lifecycle list`

### Robot doesn't move with /cmd_vel

- Make sure Nav2's velocity smoother isn't overriding commands
- Check if collision monitor is blocking movement
