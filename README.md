# selfpatch_demos

[![CI](https://github.com/selfpatch/selfpatch_demos/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/selfpatch_demos/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Discord](https://img.shields.io/badge/Discord-Join%20Us-7289DA?logo=discord&logoColor=white)](https://discord.gg/fEbWKTah)

Demonstration projects showcasing [ros2_medkit](https://github.com/selfpatch/ros2_medkit) integration
with real ROS 2 systems.

## Overview

This repository contains example integrations and demos that show how ros2_medkit
can be used to add SOVD-compliant diagnostics and fault management to ROS 2-based robots and systems.

Each demo builds on real-world scenarios, progressing from simple sensor monitoring
to complete mobile robot integration:

- **Sensor Diagnostics** — Lightweight demo focusing on data monitoring and fault injection
- **TurtleBot3 Integration** — Full-featured demo with Nav2 navigation, showing entity hierarchy and real-time control

**Key Capabilities Demonstrated:**

- ✅ SOVD-compliant REST API (Areas → Components → Apps → Functions)
- ✅ Real-time data access (topics via HTTP)
- ✅ Configuration management (ROS 2 parameters via HTTP)
- ✅ Operation execution (services and actions via HTTP)
- ✅ Fault management and injection
- ✅ Manifest-based entity discovery
- ✅ Legacy diagnostics bridge support

Both demos support:

- REST API access via SOVD protocol
- Web UI for visualization ([sovd_web_ui](https://github.com/selfpatch/sovd_web_ui))
- Fault injection and monitoring
- Docker deployment for easy setup

## Demos

| Demo | Description | Features | Status |
|------|-------------|----------|--------|
| [Sensor Diagnostics](demos/sensor_diagnostics/) | Lightweight sensor diagnostics demo (no Gazebo required) | Data monitoring, fault injection, dual fault reporting paths | ✅ Ready |
| [TurtleBot3 Integration](demos/turtlebot3_integration/) | Full ros2_medkit integration with TurtleBot3 and Nav2 | SOVD-compliant API, manifest-based discovery, fault management | ✅ Ready |

### Quick Start

#### Sensor Diagnostics Demo (Fastest - No GPU Required)

The sensor diagnostics demo is the fastest way to try ros2_medkit:

```bash
cd demos/sensor_diagnostics
./run-demo.sh
# Docker services start in daemon mode
# Web UI available at http://localhost:3000

# Explore the API
./check-demo.sh
```

**Options:**
```bash
./run-demo.sh --attached      # Run in foreground with logs
./check-demo.sh               # Interactive API demonstration
./stop-demo.sh                # Stop the demo
```

**Features:**

- Simulated sensors (LiDAR, IMU, GPS, Camera)
- Configurable fault injection via REST API
- Dual fault reporting paths (legacy + modern)
- Runs anywhere (CI, Codespaces, laptop)

#### TurtleBot3 + Nav2 Demo (Full Navigation Stack)

Full mobile robot demo with autonomous navigation:

```bash
cd demos/turtlebot3_integration
./run-demo.sh
# Gazebo will open, Web UI at http://localhost:3000
# Try: ./send-nav-goal.sh 2.0 0.5

# To stop
./stop-demo.sh
```

**Features:**

- Complete TurtleBot3 simulation in Gazebo
- Nav2 navigation stack integration
- SOVD-compliant REST API with entity hierarchy
- Manifest-based discovery (Areas → Components → Apps → Functions)
- Fault injection scenarios for Nav2 components
- Real-time robot control via HTTP

## Getting Started

### Prerequisites

- ROS 2 Jazzy (Ubuntu 24.04)
- Docker and docker-compose (recommended)
- [ros2_medkit](https://github.com/selfpatch/ros2_medkit) >= 1.0.0

### Clone the Repository

```bash
git clone https://github.com/selfpatch/selfpatch_demos.git
cd selfpatch_demos
```

### Run a Demo

Each demo has its own README with specific instructions. See above Quick Start,
or follow the detailed README in each demo directory:

```bash
cd demos/sensor_diagnostics  # or turtlebot3_integration
# Follow the README.md in that directory
```

## Example API Usage

All demos expose a SOVD-compliant REST API. Here are some common operations:

```bash
# Check gateway health
curl http://localhost:8080/api/v1/health

# List all apps (ROS 2 nodes)
curl http://localhost:8080/api/v1/apps | jq '.items[] | {id, name}'

# Get sensor data
curl http://localhost:8080/api/v1/apps/lidar_sim/data/scan | jq

# Update configuration
curl -X PUT http://localhost:8080/api/v1/apps/lidar_sim/configurations/noise_stddev \
  -H "Content-Type: application/json" \
  -d '{"value": 0.5}'

# List active faults
curl http://localhost:8080/api/v1/faults | jq
```

See individual demo READMEs for more examples.

## Related Projects

- [ros2_medkit](https://github.com/selfpatch/ros2_medkit) — Core diagnostics library with SOVD-compliant gateway
- [sovd_web_ui](https://github.com/selfpatch/sovd_web_ui) — Web-based visualization and control interface
- [ros2_medkit_mcp](https://github.com/selfpatch/ros2_medkit_mcp) — MCP server for LLM integration
- [ros2_medkit documentation](https://selfpatch.github.io/ros2_medkit/) — Full documentation and API reference

## Contributing

Contributions are welcome! Please read [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines.

By contributing, you agree to follow the [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).

## Security

If you discover a security vulnerability, please follow the process in [`SECURITY.md`](SECURITY.md).

## License

This project is licensed under the Apache License 2.0. See the [`LICENSE`](LICENSE) file for details.
