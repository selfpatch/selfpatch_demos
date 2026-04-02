# Multi-ECU Aggregation Demo

Multi-ECU diagnostics demo for **ros2_medkit** - 3 independent ECUs with peer aggregation, no Gazebo required!

This demo showcases ros2_medkit's peer aggregation feature with 3 ECU containers (perception, planning, actuation) running separate ROS 2 domains. The perception ECU acts as the aggregator, merging entity models from planning (via static peer URL) and actuation (via mDNS auto-discovery) into a unified SOVD view.

## Prerequisites

The host-side scripts (`inject-*.sh`, `restore-normal.sh`) require `curl` and `jq` to be installed on your machine.

- **Docker** (with Docker Compose)
- **curl**
- **jq**

## Quick Start

```bash
# Start the demo (builds 3 ECU containers + web UI)
./run-demo.sh

# Wait ~30s for all ECUs to boot, then verify aggregation
curl http://localhost:8080/api/v1/health | jq '.peers'

# Expected: 2 peers (planning-ecu via static URL, actuation-ecu via mDNS)

# Inject faults
./inject-sensor-failure.sh     # LiDAR failure on Perception ECU
./inject-planning-delay.sh     # Path planning delay on Planning ECU
./inject-gripper-jam.sh        # Gripper jam on Actuation ECU
./inject-cascade-failure.sh    # All of the above at once
./restore-normal.sh            # Reset everything

# Stop the demo
./stop-demo.sh
```

**Web UI:** Open http://localhost:3000 to browse the aggregated entity tree.

## Architecture

```
                            Docker Network (medkit-net)
 ┌──────────────────────────────────────────────────────────────────────────┐
 │                                                                          │
 │  ┌─────────────────────┐   static URL   ┌──────────────────────┐        │
 │  │  Perception ECU      │◄──────────────│  Planning ECU         │        │
 │  │  (ROS_DOMAIN_ID=10)  │               │  (ROS_DOMAIN_ID=20)  │        │
 │  │                      │               │                      │        │
 │  │  lidar_driver        │   detections  │  path_planner        │        │
 │  │  camera_driver       │──────────────►│  behavior_planner    │        │
 │  │  point_cloud_filter  │  domain_bridge│  task_scheduler      │        │
 │  │  object_detector     │               │  domain_bridge       │        │
 │  │                      │               │                      │        │
 │  │  Gateway :8080 ◄─────┼──── Aggregator│  Gateway :8080       │        │
 │  │  (port exposed)      │               │  (internal only)     │        │
 │  └─────────────────────┘               └──────────┬───────────┘        │
 │           ▲                                        │                    │
 │           │ mDNS discover                          │ commands           │
 │           │                                        │ domain_bridge      │
 │  ┌────────┴────────────┐                           ▼                    │
 │  │  Actuation ECU       │◄─────────────────────────┘                    │
 │  │  (ROS_DOMAIN_ID=30)  │                                               │
 │  │                      │                                               │
 │  │  motor_controller    │                                               │
 │  │  joint_driver        │                                               │
 │  │  gripper_controller  │                                               │
 │  │  domain_bridge       │                                               │
 │  │                      │                                               │
 │  │  Gateway :8080       │                                               │
 │  │  (mDNS announce)     │                                               │
 │  └─────────────────────┘                                               │
 │                                                                          │
 │  ┌─────────────────────┐                                                │
 │  │  Web UI :3000        │──── connects to Perception ECU gateway        │
 │  └─────────────────────┘                                                │
 └──────────────────────────────────────────────────────────────────────────┘
                │
                ▼
         Host: localhost:8080 (API), localhost:3000 (UI)
```

| Container | ROS_DOMAIN_ID | Role | Port |
|-----------|:---:|------|------|
| **perception-ecu** | 10 | Aggregator - pulls from peers | 8080 (exposed to host) |
| **planning-ecu** | 20 | Peer - discovered via static URL | 8080 (internal) |
| **actuation-ecu** | 30 | Peer - discovered via mDNS announce | 8080 (internal) |
| **medkit-web-ui** | - | Entity browser UI | 3000 (exposed to host) |

## DDS Isolation

Each ECU runs in its own DDS domain to simulate physically separate compute units:

- **Domain 10** - Perception ECU: LiDAR scans, camera frames, detections
- **Domain 20** - Planning ECU: path plans, behavior commands, task schedules
- **Domain 30** - Actuation ECU: motor status, joint positions, gripper state

Cross-ECU data flows use `ros2_domain_bridge` nodes to relay topics between domains:

| Bridge | From | To | Topic | Type |
|--------|------|----|-------|------|
| planning_bridge | Domain 10 | Domain 20 | `/perception/detections` | `vision_msgs/msg/Detection2DArray` |
| actuation_bridge | Domain 20 | Domain 30 | `/planning/commands` | `geometry_msgs/msg/Twist` |

This mirrors a real multi-ECU system where each compute unit has its own DDS network and selected topics are explicitly bridged across domains.

## Entity Hierarchy (SOVD)

The aggregated entity model (as seen from the perception ECU gateway at `:8080`) merges entities from all 3 ECUs:

### Components

```
robot-alpha              (parent - mobile-robot)
├── perception-ecu       (local)
├── planning-ecu         (from planning peer)
└── actuation-ecu        (from actuation peer)
```

All 3 manifests declare `robot-alpha` as the parent component. The aggregator merges them by component ID, creating a single parent with 3 sub-components.

### Apps (10 total)

| App | ECU | Category |
|-----|-----|----------|
| lidar-driver | Perception | sensor |
| camera-driver | Perception | sensor |
| point-cloud-filter | Perception | processing |
| object-detector | Perception | processing |
| path-planner | Planning | planning |
| behavior-planner | Planning | planning |
| task-scheduler | Planning | planning |
| motor-controller | Actuation | actuation |
| joint-driver | Actuation | actuation |
| gripper-controller | Actuation | actuation |

### Functions (3 cross-ECU)

Functions with the same ID across ECUs are merged. The aggregator combines `hosted_by` lists from all peers:

| Function | Category | Perception Apps | Planning Apps | Actuation Apps |
|----------|----------|-----------------|---------------|----------------|
| **autonomous-navigation** | navigation | object-detector, point-cloud-filter | path-planner, behavior-planner | motor-controller, joint-driver |
| **obstacle-avoidance** | safety | lidar-driver, object-detector | - | motor-controller, gripper-controller |
| **task-execution** | execution | - | task-scheduler, behavior-planner | motor-controller, joint-driver, gripper-controller |

This demonstrates how SOVD functions provide a **functional view** spanning multiple ECUs - a fault in any participating app degrades the function.

## Discovery Mechanisms

The perception ECU aggregator uses two complementary discovery mechanisms to find its peers:

### Static Peer URLs (Planning ECU)

The perception gateway config explicitly lists the planning ECU:

```yaml
aggregation:
  peer_urls: ["http://planning-ecu:8080"]
  peer_names: ["planning-ecu"]
```

This is the simplest approach - hardcode peer addresses when the network topology is known.

### mDNS Auto-Discovery (Actuation ECU)

The actuation ECU announces itself via mDNS:

```yaml
# actuation_params.yaml
aggregation:
  announce: true
  mdns_service: "_medkit._tcp.local"
```

The perception ECU discovers it by listening for mDNS announcements:

```yaml
# perception_params.yaml
aggregation:
  discover: true
  mdns_service: "_medkit._tcp.local"
```

### Verifying Peer Discovery

```bash
# Check which peers the aggregator has discovered
curl http://localhost:8080/api/v1/health | jq '.peers'

# Expected output:
# [
#   { "name": "planning-ecu", "url": "http://planning-ecu:8080", "status": "connected" },
#   { "name": "actuation-ecu", "url": "http://actuation-ecu:8080", "status": "connected" }
# ]
```

## REST API Examples

All requests go through the perception ECU gateway at `localhost:8080`, which transparently proxies to remote ECUs.

### Health and Peers

```bash
# Gateway health with peer status
curl http://localhost:8080/api/v1/health | jq

# Peer list
curl http://localhost:8080/api/v1/health | jq '.peers'
```

### Components

```bash
# All components (merged from 3 ECUs)
curl http://localhost:8080/api/v1/components | jq

# Parent component
curl http://localhost:8080/api/v1/components/robot-alpha | jq

# Sub-components (3 ECUs)
curl http://localhost:8080/api/v1/components/robot-alpha/subcomponents | jq
```

### Apps

```bash
# All 10 apps from all ECUs
curl http://localhost:8080/api/v1/apps | jq

# Local app (perception ECU)
curl http://localhost:8080/api/v1/apps/lidar-driver | jq

# Remote app (transparently proxied to planning ECU)
curl http://localhost:8080/api/v1/apps/path-planner | jq

# Remote app data (proxied to actuation ECU)
curl http://localhost:8080/api/v1/apps/motor-controller/data | jq
```

### Functions (Cross-ECU)

```bash
# All merged functions
curl http://localhost:8080/api/v1/functions | jq

# Function detail - shows apps from all 3 ECUs
curl http://localhost:8080/api/v1/functions/autonomous-navigation | jq
```

### Faults

```bash
# Aggregated faults from all ECUs
curl http://localhost:8080/api/v1/faults | jq

# Faults for a specific component
curl http://localhost:8080/api/v1/components/perception-ecu/faults | jq
```

## Fault Injection Scenarios

### Scripts

| Script | Target ECU | Effect | Affected Functions |
|--------|-----------|--------|-------------------|
| `inject-sensor-failure.sh` | Perception | LiDAR failure (high failure probability) | autonomous-navigation, obstacle-avoidance |
| `inject-planning-delay.sh` | Planning | Path planner delay (5000ms processing) | autonomous-navigation |
| `inject-gripper-jam.sh` | Actuation | Gripper jam (stuck closed) | obstacle-avoidance, task-execution |
| `inject-cascade-failure.sh` | All | All of the above combined | All 3 functions degraded |
| `restore-normal.sh` | All | Reset parameters, clear faults | All restored |

### Demo Walkthrough

1. **Start the demo** and wait for all peers to connect:
   ```bash
   ./run-demo.sh
   # Wait ~30s, then verify:
   curl http://localhost:8080/api/v1/health | jq '.peers'
   ```

2. **Verify the aggregated entity model** - 3 sub-components, 10 apps, 3 functions:
   ```bash
   curl http://localhost:8080/api/v1/components/robot-alpha/subcomponents | jq '.items | length'
   curl http://localhost:8080/api/v1/apps | jq '.items | length'
   curl http://localhost:8080/api/v1/functions | jq '.items | length'
   ```

3. **Inject a single-ECU fault** and observe how it degrades cross-ECU functions:
   ```bash
   ./inject-sensor-failure.sh
   curl http://localhost:8080/api/v1/faults | jq
   curl http://localhost:8080/api/v1/functions/autonomous-navigation | jq
   ```

4. **Restore and inject a cascade failure** across all ECUs:
   ```bash
   ./restore-normal.sh
   ./inject-cascade-failure.sh
   curl http://localhost:8080/api/v1/faults | jq
   ```

5. **Verify all 3 functions are degraded**:
   ```bash
   curl http://localhost:8080/api/v1/functions | jq
   ```

6. **Restore normal operation**:
   ```bash
   ./restore-normal.sh
   curl http://localhost:8080/api/v1/faults | jq '.items | length'
   # Expected: 0
   ```

## Container Scripts (Scripts API)

Each ECU has container-side scripts callable via the gateway REST API. The aggregator transparently proxies script execution to remote ECUs.

### List Available Scripts

```bash
# Perception ECU scripts
curl http://localhost:8080/api/v1/components/perception-ecu/scripts | jq

# Planning ECU scripts (proxied through aggregator)
curl http://localhost:8080/api/v1/components/planning-ecu/scripts | jq

# Actuation ECU scripts (proxied through aggregator)
curl http://localhost:8080/api/v1/components/actuation-ecu/scripts | jq
```

### Execute a Script

```bash
# Execute inject-gripper-jam on actuation ECU (proxied)
curl -X POST http://localhost:8080/api/v1/components/actuation-ecu/scripts/inject-gripper-jam/executions \
  -H "Content-Type: application/json" \
  -d '{"execution_type":"now"}' | jq
```

### Available Scripts per ECU

| ECU | Script | Description |
|-----|--------|-------------|
| **perception-ecu** | `inject-sensor-failure` | Inject LiDAR failure (high failure probability) |
| **perception-ecu** | `restore-normal` | Reset perception parameters and clear faults |
| **planning-ecu** | `inject-planning-delay` | Inject path planning delay (5000ms) |
| **planning-ecu** | `restore-normal` | Reset planning parameters and clear faults |
| **actuation-ecu** | `inject-gripper-jam` | Inject gripper jam (stuck) |
| **actuation-ecu** | `restore-normal` | Reset actuation parameters and clear faults |

### Override Gateway URL

```bash
# Point host-side scripts at a non-default gateway
GATEWAY_URL=http://192.168.1.10:8080 ./inject-sensor-failure.sh
```

## Options

### run-demo.sh

```bash
./run-demo.sh                # Daemon mode (default)
./run-demo.sh --attached     # Run in foreground with logs
./run-demo.sh --update       # Pull latest images before running
./run-demo.sh --no-cache     # Build Docker images without cache
```

### stop-demo.sh

```bash
./stop-demo.sh               # Stop containers
./stop-demo.sh --volumes     # Stop and remove named volumes
./stop-demo.sh --images      # Stop and remove images
```

## Scripts

| Script | Description |
|--------|-------------|
| `run-demo.sh` | Start Docker services (3 ECUs + web UI) |
| `stop-demo.sh` | Stop Docker services |
| `inject-sensor-failure.sh` | Inject LiDAR sensor failure on Perception ECU |
| `inject-planning-delay.sh` | Inject path planning delay on Planning ECU |
| `inject-gripper-jam.sh` | Inject gripper jam on Actuation ECU |
| `inject-cascade-failure.sh` | Inject faults across all 3 ECUs |
| `restore-normal.sh` | Reset all ECUs and clear faults |

> **Note:** All fault injection and restore scripts are also available via the [Scripts API](#container-scripts-scripts-api) - callable as REST endpoints without requiring the host-side scripts.

## Troubleshooting

### mDNS Discovery Not Working

The actuation ECU uses mDNS to announce itself to the perception aggregator. This requires multicast to work on the Docker bridge network.

- Verify the Docker network supports multicast (the default `bridge` driver does)
- Check that the `medkit-net` network was created: `docker network ls | grep medkit`
- Inspect mDNS traffic: `docker exec perception_ecu avahi-browse -at` (if avahi-utils is installed)
- Fallback: add `actuation-ecu` to `peer_urls` in `perception_params.yaml` as a static peer

### Gateway Startup Order

The perception ECU aggregator starts immediately but peers may take a few seconds to boot. Initial requests may show fewer entities until all peers connect.

- The aggregator retries peer connections periodically
- Wait 20-30 seconds after `./run-demo.sh` before verifying
- Check peer status: `curl http://localhost:8080/api/v1/health | jq '.peers'`

### Port Conflicts

The demo exposes port 8080 (gateway) and 3000 (web UI) on the host.

- If port 8080 is in use: change the port mapping in `docker-compose.yml` under `perception-ecu.ports`
- If port 3000 is in use: change the port mapping under `medkit-web-ui.ports`
- Internal gateway ports (8080 on planning/actuation) are not exposed to the host

### Containers Not Starting

```bash
# Check container status
docker compose ps

# View logs for a specific ECU
docker compose logs perception-ecu
docker compose logs planning-ecu
docker compose logs actuation-ecu

# Rebuild from scratch
./stop-demo.sh --images
./run-demo.sh --no-cache
```

## License

Apache 2.0 - See [LICENSE](../../LICENSE)
