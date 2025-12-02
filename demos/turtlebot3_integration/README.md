# TurtleBot3 Integration Demo with Nav2 Navigation

This demo shows how to integrate ros2_medkit with TurtleBot3 and Nav2 navigation stack
to provide modern diagnostics and control for a mobile robot system via REST API.

## Status

✅ **Demo Ready** - Full navigation demo with Web UI

## Overview

This demo demonstrates:

- Launching TurtleBot3 simulation in Gazebo with turtlebot3_world
- Running Nav2 navigation stack (AMCL, planner, controller)
- Running ros2_medkit gateway alongside the robot
- Discovering TurtleBot3 nodes through REST API
- Querying and publishing to ROS2 topics via HTTP
- **NEW:** Controlling the robot via sovd_web_ui

## Prerequisites

- Docker and docker-compose
- X11 display server (Linux with GUI, or XQuartz on macOS)
- (Optional) NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

## Quick Start

### 1. Start the ROS2 Backend

```bash
cd demos/turtlebot3_integration
./run-demo.sh
```

That's it! The script will:

1. Build the Docker images (first run takes ~5-10 min, downloads ~4GB)
2. Setup X11 forwarding for Gazebo GUI
3. Launch TurtleBot3 simulation + Nav2 + ros2_medkit gateway
4. Launch sovd_web_ui at <http://localhost:3000>

### 2. Access the Web UI

The Web UI is automatically started by docker-compose and available at <http://localhost:3000>.

Connect to the gateway using `http://localhost:8080/api/v1` in the connection dialog.

**Note:** The first build will take longer as it clones and builds sovd_web_ui from GitHub.

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
docker compose --profile nvidia up --build
```

## Controlling the Robot

### Via Web UI

1. Connect to the gateway in sovd_web_ui
2. In the "ROS2 Topics" panel on the right, select `/cmd_vel`
3. Enter velocity command JSON:

   ```json
   {"linear": {"x": 0.2}, "angular": {"z": 0.0}}
   ```

4. Click "Send" - the robot will move!

### Via Command Line

```bash
# Send velocity command (moves robot forward)
curl -X POST http://localhost:8080/api/v1/topics/publish \
  -H "Content-Type: application/json" \
  -d '{
    "topic": "/cmd_vel",
    "type": "geometry_msgs/msg/Twist",
    "data": {"linear": {"x": 0.2, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
  }'

# Stop the robot
curl -X POST http://localhost:8080/api/v1/topics/publish \
  -H "Content-Type: application/json" \
  -d '{
    "topic": "/cmd_vel",
    "type": "geometry_msgs/msg/Twist",
    "data": {"linear": {"x": 0.0}, "angular": {"z": 0.0}}
  }'
```

### Via ROS2 CLI (inside container)

```bash
# Send navigation goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}"

# Manual teleop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

## REST API Endpoints

### Discovery

```bash
# Check gateway health
curl http://localhost:8080/api/v1/health

# List discovered areas
curl http://localhost:8080/api/v1/areas

# List all discovered components (nodes)
curl http://localhost:8080/api/v1/components
```

### Topics

```bash
# List all topics
curl http://localhost:8080/api/v1/topics

# Get topic details (URL-encode topic name: / -> %2F)
curl http://localhost:8080/api/v1/topics/%2Fcmd_vel

# Get topic without sample
curl "http://localhost:8080/api/v1/topics/%2Fcmd_vel?sample=false"

# Publish to topic (see examples above)
curl -X POST http://localhost:8080/api/v1/topics/publish ...
```

## What You'll See

When TurtleBot3 simulation starts with Nav2, ros2_medkit will discover nodes such as:

- `turtlebot3_node` - Main robot interface
- `robot_state_publisher` - TF tree publisher
- `gazebo` - Simulation engine
- `amcl` - Adaptive Monte Carlo Localization
- `bt_navigator` - Behavior Tree Navigator
- `controller_server` - Path following controller
- `planner_server` - Global path planner
- Various sensor and lifecycle nodes

These appear as **components** in the ros2_medkit REST API, organized into **areas** based on their ROS 2 namespaces.

## Architecture

```text
┌─────────────────────────────────────────────────────────────────────┐
│                       Docker Container                               │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │                   Gazebo Simulation                            │   │
│  │  ┌─────────────┐  ┌──────────────┐  ┌────────┐ ┌───────────┐  │   │
│  │  │ TurtleBot3  │  │ robot_state  │  │ LIDAR  │ │   Nav2    │  │   │
│  │  │    Node     │  │  publisher   │  │ sensor │ │ (AMCL,    │  │   │
│  │  └──────┬──────┘  └──────┬───────┘  └───┬────┘ │ Planner,  │  │   │
│  │         │                │              │      │ Controller│  │   │
│  │         └────────────────┼──────────────┘      └─────┬─────┘  │   │
│  │                    ROS 2 Topics ◄────────────────────┘        │   │
│  └──────────────────────────┼────────────────────────────────────┘   │
│                             │                                        │
│               ┌─────────────┴─────────────┐                         │
│               │   ros2_medkit Gateway     │                         │
│               │   (REST Server :8080)     │                         │
│               └─────────────┬─────────────┘                         │
└─────────────────────────────┼────────────────────────────────────────┘
                              │
                       HTTP REST API
                              │
         ┌────────────────────┼────────────────────┐
         ▼                    ▼                    ▼
   sovd_web_ui          curl/browser        External tools
  (localhost:3000)
```

## File Structure

```text
demos/turtlebot3_integration/
├── Dockerfile                  # ROS 2 Jazzy + TurtleBot3 + Nav2 + ros2_medkit
├── docker-compose.yml          # Docker Compose (CPU & GPU via profiles)
├── run-demo.sh                 # One-click demo launcher
├── config/
│   ├── medkit_params.yaml      # ros2_medkit gateway config
│   ├── nav2_params.yaml        # Nav2 navigation parameters
│   └── turtlebot3_world.yaml   # Map configuration
└── launch/
    └── demo.launch.py          # ROS 2 launch file
```

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
