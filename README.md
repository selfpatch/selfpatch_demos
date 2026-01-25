# selfpatch_demos

[![CI](https://github.com/selfpatch/selfpatch_demos/actions/workflows/ci.yml/badge.svg)](https://github.com/selfpatch/selfpatch_demos/actions/workflows/ci.yml)

Demonstration projects showcasing [ros2_medkit](https://github.com/selfpatch/ros2_medkit) integration
with real ROS 2 systems.

## Overview

This repository contains example integrations and demos that show how ros2_medkit
can be used to add modern diagnostics to ROS 2-based robots and systems.

Each demo builds on real-world scenarios, starting from basic integration and
progressing toward more advanced use cases.

## Demos

| Demo | Description | Status |
|------|-------------|--------|
| [Sensor Diagnostics](demos/sensor_diagnostics/) | Lightweight sensor diagnostics demo (no Gazebo required) | ✅ Ready |
| [TurtleBot3 Integration](demos/turtlebot3_integration/) | Full ros2_medkit integration with TurtleBot3 and Nav2 | 🚧 In Progress |

### Quick Start (Sensor Diagnostics)

The sensor diagnostics demo is the fastest way to try ros2_medkit:

```bash
cd demos/sensor_diagnostics
docker compose up
# Open http://localhost:3000 for the Web UI
# Run ./run-demo.sh for an interactive walkthrough
```

## Getting Started

### Prerequisites

- ROS 2 Jazzy (Ubuntu 24.04)
- [ros2_medkit](https://github.com/selfpatch/ros2_medkit) installed

### Clone the Repository

```bash
git clone https://github.com/selfpatch/selfpatch_demos.git
cd selfpatch_demos
```

### Run a Demo

Each demo has its own README with specific instructions. Start with:

```bash
cd demos/turtlebot3_integration
# Follow the README.md in that directory
```

## Related Projects

- [ros2_medkit](https://github.com/selfpatch/ros2_medkit) — The core diagnostics library
- [ros2_medkit documentation](https://selfpatch.github.io/ros2_medkit/) — Full documentation and API reference

## Contributing

Contributions are welcome! Please read [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines.

By contributing, you agree to follow the [`CODE_OF_CONDUCT.md`](CODE_OF_CONDUCT.md).

## Security

If you discover a security vulnerability, please follow the process in [`SECURITY.md`](SECURITY.md).

## License

This project is licensed under the Apache License 2.0. See the [`LICENSE`](LICENSE) file for details.