# selfpatch_demos

[![CI](https://github.com/selfpatch/selfpatch_demos/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/selfpatch_demos/actions/workflows/ci.yml)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue)](https://selfpatch.github.io/ros2_medkit/)
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Discord](https://img.shields.io/badge/Discord-Join%20Us-7289DA?logo=discord&logoColor=white)](https://discord.gg/6CXPMApAyq)

Demonstration projects showcasing [ros2_medkit](https://github.com/selfpatch/ros2_medkit) integration
with real ROS 2 systems.

## Overview

This repository contains example integrations and demos that show how ros2_medkit
can be used to add SOVD-compliant diagnostics and fault management to ROS 2-based robots and systems.

Each demo builds on real-world scenarios, progressing from simple sensor monitoring
to complete mobile robot integration and multi-ECU peer aggregation:

- **Sensor Diagnostics** - Lightweight demo focusing on data monitoring and fault injection
- **TurtleBot3 Integration** - Full-featured demo with Nav2 navigation, showing entity hierarchy and real-time control
- **MoveIt Pick-and-Place** - Panda 7-DOF arm manipulation with MoveIt 2, fault monitoring for planning, controllers, and joint limits
- **Multi-ECU Aggregation** - Peer aggregation with 3 ECUs (perception, planning, actuation), mDNS discovery, and cross-ECU functions

**Key Capabilities Demonstrated:**

- ✅ SOVD-compliant REST API (Areas → Components → Apps → Functions)
- ✅ Real-time data access (topics via HTTP)
- ✅ Configuration management (ROS 2 parameters via HTTP)
- ✅ Operation execution (services and actions via HTTP)
- ✅ Fault management and injection
- ✅ Manifest-based entity discovery
- ✅ Legacy diagnostics bridge support
- ✅ Multi-ECU peer aggregation
- ✅ mDNS-based ECU discovery
- ✅ Cross-ECU function grouping

All demos support:

- REST API access via SOVD protocol
- Web UI for visualization ([ros2_medkit_web_ui](https://github.com/selfpatch/ros2_medkit_web_ui))
- Fault injection and monitoring
- Docker deployment for easy setup

## Demos

| Demo | Description | Features | Status |
|------|-------------|----------|--------|
| [Sensor Diagnostics](demos/sensor_diagnostics/) | Lightweight sensor diagnostics demo (no Gazebo required) | Data monitoring, fault injection, dual fault reporting paths | ✅ Ready |
| [TurtleBot3 Integration](demos/turtlebot3_integration/) | Full ros2_medkit integration with TurtleBot3 and Nav2 | SOVD-compliant API, manifest-based discovery, fault management | ✅ Ready |
| [MoveIt Pick-and-Place](demos/moveit_pick_place/) | Panda 7-DOF arm with MoveIt 2 manipulation and ros2_medkit | Planning fault detection, controller monitoring, joint limits | ✅ Ready |
| [Multi-ECU Aggregation](demos/multi_ecu_aggregation/) | Multi-ECU peer aggregation with 3 ECUs (perception, planning, actuation), mDNS discovery, cross-ECU functions | Peer aggregation, mDNS discovery, cross-ECU functions | ✅ Ready |
| [OTA over SOVD - nav2 sensor fix](demos/ota_nav2_sensor_fix/) | Dev-grade OTA plugin showing the SOVD `/updates` lifecycle - update a broken lidar node, install a new safety classifier, uninstall a deprecated package | SOVD-spec update / install / uninstall, native binary swap, fork+exec process management, Foxglove panel + curl scripts | ✅ Ready |

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

#### MoveIt 2 Pick-and-Place Demo (Manipulation Stack)

Panda robot arm demo with pick-and-place manipulation:

```bash
cd demos/moveit_pick_place
./run-demo.sh
# RViz will open with Panda arm (or use --headless), Web UI at http://localhost:3000
# Move the arm: ./move-arm.sh demo
# Inject faults: ./inject-planning-failure.sh
# Check faults: ./check-faults.sh

# To stop
./stop-demo.sh
```

**Features:**

- Panda 7-DOF arm with MoveIt 2 and mock hardware (no physics sim)
- Interactive arm control via `move-arm.sh`
- Continuous pick-and-place task loop
- Manipulation fault monitoring (planning, controller, joint limits)
- 5 fault injection scenarios with one-click scripts
- SOVD-compliant REST API with rich entity hierarchy (4 areas, 7 components)

#### Multi-ECU Aggregation Demo (Advanced - Peer Aggregation)

Multi-ECU demo with 3 independent ECUs aggregated via mDNS discovery:

```bash
cd demos/multi_ecu_aggregation
./run-demo.sh
./check-demo.sh  # Verify all 3 ECUs are connected
# Gateway at http://localhost:8080, Web UI at http://localhost:3000

# To stop
./stop-demo.sh
```

**Features:**

- 3 independent ECUs (perception, planning, actuation) each running ros2_medkit
- Peer aggregation via mDNS-based automatic ECU discovery
- Cross-ECU function grouping across the full system
- Unified SOVD-compliant REST API spanning all ECUs
- Web UI for browsing aggregated entity hierarchy

#### OTA over SOVD Demo (Dev-grade Update / Install / Uninstall)

End-to-end demo of the SOVD `/updates` resource: a broken lidar node is
swapped with a fixed version over HTTP, an extra safety classifier is
installed from scratch, and a deprecated package is uninstalled - all
without SSH, all spec-compliant.

```bash
cd demos/ota_nav2_sensor_fix
./run-demo.sh                # build artifacts + bring up gateway/plugin/update server
./check-demo.sh              # show registered updates + per-id status + live process state
./trigger-update.sh          # broken_lidar -> fixed_lidar (the headline)
./trigger-install.sh         # install obstacle_classifier_v2
./trigger-uninstall.sh       # remove broken_lidar_legacy
./stop-demo.sh
```

**Features:**

- Dev-grade `ota_update_plugin` C++ gateway plugin (UpdateProvider + GatewayPlugin)
- SOVD ISO 17978-3 compliant `/updates` resource: kind derived from
  `updated_components` / `added_components` / `removed_components` metadata
- Native binary swap + `fork+exec` process management (no containers, no signing)
- Foxglove Studio panel mirrors the same SOVD client patterns as the web UI
- Pairs with the [`ros2_medkit_foxglove_extension`](https://github.com/selfpatch/ros2_medkit_foxglove_extension) Updates panel

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
cd demos/sensor_diagnostics  # or turtlebot3_integration, moveit_pick_place, multi_ecu_aggregation
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

## Testing

Each demo has automated smoke tests that verify the gateway starts and the REST API works correctly:

```bash
# Run smoke tests against a running demo (default: http://localhost:8080)
./tests/smoke_test.sh              # Sensor diagnostics (full API coverage + fault injection + beacons)
./tests/smoke_test_turtlebot3.sh   # TurtleBot3 (discovery, data, operations, scripts, triggers, logs)
./tests/smoke_test_moveit.sh       # MoveIt pick-and-place (discovery, data, operations, scripts, triggers, logs)
./tests/smoke_test_multi_ecu.sh    # Multi-ECU aggregation (per-ECU discovery + aggregated view)
./tests/smoke_test_ota.sh          # OTA over SOVD (catalog, /updates spec shape, prepare/execute, process swap)
```

CI runs all demos in parallel - each job builds the Docker image, starts the container, and runs the smoke tests against it. See [CI workflow](.github/workflows/ci.yml).

## Related Projects

- [ros2_medkit](https://github.com/selfpatch/ros2_medkit) — Core diagnostics library with SOVD-compliant gateway
- [ros2_medkit_web_ui](https://github.com/selfpatch/ros2_medkit_web_ui) — Web-based visualization and control interface
- [ros2_medkit_mcp](https://github.com/selfpatch/ros2_medkit_mcp) — MCP server for LLM integration
- [ros2_medkit documentation](https://selfpatch.github.io/ros2_medkit/) — Full documentation and API reference

## Contributing

Contributions are welcome! Please read [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines.

By contributing, you agree to follow the [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).

## Security

If you discover a security vulnerability, please follow the process in [`SECURITY.md`](SECURITY.md).

## License

This project is licensed under the Apache License 2.0. See the [`LICENSE`](LICENSE) file for details.
