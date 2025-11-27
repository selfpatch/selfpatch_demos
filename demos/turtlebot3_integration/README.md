# TurtleBot3 Integration Demo

This demo shows how to integrate ros2_medkit with TurtleBot3 to provide
modern diagnostics for a mobile robot system via REST API.

## Status

🚧 **Work in Progress** - Basic discovery demo

## Overview

This demo demonstrates:

- Launching TurtleBot3 simulation in Gazebo
- Running ros2_medkit gateway alongside the robot
- Discovering TurtleBot3 nodes through REST API
- Querying robot components and their data via HTTP

## Prerequisites

- Docker and docker-compose
- X11 display server (Linux with GUI, or XQuartz on macOS)
- (Optional) NVIDIA GPU + [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

## Quick Start

```bash
cd demos/turtlebot3_integration
./run-demo.sh
```

That's it! The script will:
1. Build the Docker image (first run takes ~5-10 min, downloads ~4GB)
2. Setup X11 forwarding for Gazebo GUI
3. Launch TurtleBot3 simulation + ros2_medkit gateway

### With NVIDIA GPU

For better Gazebo performance:

```bash
./run-demo.sh --nvidia
```

Requires [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html).

## Verifying Discovery

Once the demo is running, open a new terminal and test the REST API:

```bash
# Check gateway health
curl http://localhost:8080/health

# List discovered areas
curl http://localhost:8080/areas

# List all discovered components (nodes)
curl http://localhost:8080/components
```

## What You'll See

When TurtleBot3 simulation starts, ros2_medkit will discover nodes such as:

- `turtlebot3_node` - Main robot interface
- `robot_state_publisher` - TF tree publisher
- `gazebo` - Simulation engine
- Various sensor and controller nodes

These appear as **components** in the ros2_medkit REST API, organized into **areas** based on their ROS 2 namespaces.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                 Docker Container                         │
│  ┌─────────────────────────────────────────────────┐   │
│  │              Gazebo Simulation                   │   │
│  │  ┌─────────────┐  ┌──────────────┐  ┌────────┐  │   │
│  │  │ TurtleBot3  │  │ robot_state  │  │ LIDAR  │  │   │
│  │  │    Node     │  │  publisher   │  │ sensor │  │   │
│  │  └──────┬──────┘  └──────┬───────┘  └───┬────┘  │   │
│  │         └────────────────┼──────────────┘       │   │
│  │                    ROS 2 Topics                 │   │
│  └──────────────────────────┼──────────────────────┘   │
│                             │                           │
│               ┌─────────────┴─────────────┐            │
│               │   ros2_medkit Gateway     │            │
│               │   (REST Server :8080)     │            │
│               └─────────────┬─────────────┘            │
└─────────────────────────────┼───────────────────────────┘
                              │
                       HTTP REST API
                              │
                 ┌────────────┴────────────┐
                 ▼                         ▼
           curl/browser            External tools
```

## File Structure

```
demos/turtlebot3_integration/
├── Dockerfile                # ROS 2 Jazzy + TurtleBot3 + ros2_medkit
├── docker-compose.yml        # Standard (CPU) configuration
├── docker-compose.nvidia.yml # NVIDIA GPU configuration
├── run-demo.sh               # One-click demo launcher
├── config/
│   └── medkit_params.yaml    # ros2_medkit gateway config
└── launch/
    └── demo.launch.py        # ROS 2 launch file
```

## Manual Setup (Alternative)

If you prefer not to use Docker:

1. Install ROS 2 Jazzy on Ubuntu 24.04
2. Install TurtleBot3: `sudo apt install ros-jazzy-turtlebot3-gazebo`
3. Build [ros2_medkit](https://github.com/selfpatch/ros2_medkit) from source
4. Set `export TURTLEBOT3_MODEL=burger`
5. Run: `ros2 launch launch/demo.launch.py`

## Next Steps

Future versions of this demo will add:

- Nav2 navigation stack integration
- Teleopereration with diagnostic monitoring
- Sensor data visualization through the API
- Health monitoring during autonomous navigation
