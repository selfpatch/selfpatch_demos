# Sensor Diagnostics Demo

Lightweight sensor diagnostics demo for **ros2_medkit** - no Gazebo required!

This demo showcases ros2_medkit's data monitoring, configuration management, and fault detection using simulated sensor nodes with configurable fault injection.

## Features

- **Runs anywhere** - No Gazebo, no GPU, works in CI and GitHub Codespaces
- **Fast startup** - Seconds vs minutes compared to TurtleBot3 demo
- **Focus on diagnostics** - Pure ros2_medkit features without robot complexity
- **Configurable faults** - Runtime fault injection via REST API
- **Dual fault reporting** - Demonstrates both legacy (diagnostics) and modern (direct) paths
- **Beacon discovery** - Optional push (topic) or pull (parameter) entity enrichment

## Quick Start

> **Host prerequisites:** The host-side scripts (`check-demo.sh`, `inject-*.sh`, `restore-normal.sh`) require `curl` and `jq` to be installed on your machine.

### Using Docker (Recommended)

```bash
# Start the demo (builds and starts Docker services)
./run-demo.sh

# The script will:
# 1. Build and start Docker containers in daemon mode
# 2. Display access URLs and available commands

# Explore the API with interactive demonstration
./check-demo.sh

# To stop the demo
./stop-demo.sh
```

**Options:**
```bash
./run-demo.sh --attached      # Run in foreground with logs
./run-demo.sh --update        # Pull latest images before running
./run-demo.sh --no-cache      # Build without cache
```

**Advanced usage:**
```bash
# Manual Docker control (if needed)
docker compose up              # Start services manually
docker compose down            # Stop services
```

### Building from Source

```bash
# In a ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/selfpatch/selfpatch_demos.git
cd ..

# Build
colcon build --packages-select sensor_diagnostics_demo

# Launch
source install/setup.bash
ros2 launch sensor_diagnostics_demo demo.launch.py
```

## Architecture

```
Sensor Diagnostics Demo
├── /sensors                    # Simulated sensor nodes
│   ├── lidar_sim              # 2D LiDAR (LaserScan) - Legacy path
│   ├── camera_sim             # RGB camera (Image) - Legacy path
│   ├── imu_sim                # 9-DOF IMU (Imu) - Modern path
│   └── gps_sim                # GPS receiver (NavSatFix) - Modern path
├── /processing                 # Data processing
│   └── anomaly_detector       # Fault detection (modern path)
├── /bridge                     # Diagnostic conversion
│   └── diagnostic_bridge      # /diagnostics → FaultManager (legacy path)
└── /diagnostics                # Monitoring
    └── ros2_medkit_gateway    # REST API gateway
```

## Fault Reporting Paths

This demo demonstrates **two different fault reporting mechanisms** available in ros2_medkit:

### Legacy Path (ROS 2 Diagnostics → Diagnostic Bridge)

Standard ROS 2 diagnostics pattern used by **LiDAR** and **Camera** sensors:

```
Sensor Node → publishes DiagnosticArray → /diagnostics topic
                                              ↓
                    diagnostic_bridge subscribes and converts
                                              ↓
                    FaultManager receives via ReportFault service
```

- Sensors publish `diagnostic_msgs/DiagnosticArray` to `/diagnostics`
- `ros2_medkit_diagnostic_bridge` converts DiagnosticStatus levels to fault severities:
  - `OK (0)` → PASSED event (fault healing)
  - `WARN (1)` → WARNING severity fault
  - `ERROR (2)` → ERROR severity fault
  - `STALE (3)` → CRITICAL severity fault

> **Note:** This demo's sensors use only `OK` and `ERROR` levels for clear fault demonstration. All non-OK conditions report as `ERROR` to ensure reliable fault detection through the diagnostic bridge.

### Modern Path (Direct ReportFault Service)

Direct ros2_medkit integration used by **IMU** and **GPS** sensors:

```
Sensor Topics → anomaly_detector monitors
                        ↓
    Detects anomalies (NaN, timeout, out-of-range)
                        ↓
    FaultManager receives via ReportFault service
```

- `anomaly_detector` subscribes to sensor topics
- Analyzes data for anomalies in real-time
- Calls `/fault_manager/report_fault` service directly

### Fault Reporting Summary

| Sensor | Reporting Path | Fault Reporter | Fault Types |
|--------|---------------|----------------|-------------|
| **LiDAR** | Legacy (diagnostics) | diagnostic_bridge | NAN_VALUES, HIGH_NOISE, DRIFTING, TIMEOUT |
| **Camera** | Legacy (diagnostics) | diagnostic_bridge | BLACK_FRAME, HIGH_NOISE, LOW_BRIGHTNESS, OVEREXPOSED, TIMEOUT |
| **IMU** | Modern (direct) | anomaly_detector | SENSOR_NAN, SENSOR_OUT_OF_RANGE, RATE_DEGRADED, SENSOR_TIMEOUT |
| **GPS** | Modern (direct) | anomaly_detector | SENSOR_NAN, NO_FIX, SENSOR_OUT_OF_RANGE, RATE_DEGRADED, SENSOR_TIMEOUT |

### Inject Scripts and Fault Paths

| Script | Target Sensor | Reporting Path | Fault Scenario |
|--------|---------------|----------------|----------------|
| `inject-nan.sh` | LiDAR, IMU, GPS | Both paths | NaN values in sensor data |
| `inject-noise.sh` | LiDAR, Camera | Legacy | High noise levels |
| `inject-drift.sh` | LiDAR | Legacy | Gradual sensor drift |
| `inject-failure.sh` | IMU | Modern | Complete sensor timeout |
| `restore-normal.sh` | All | Both | Clears all faults |

## Scripts API

The inject and restore scripts run inside the container and are also callable directly via the gateway REST API using the Scripts endpoint. This lets you trigger fault scenarios programmatically without needing the shell scripts on the host.

### List Available Scripts

```bash
curl http://localhost:8080/api/v1/components/compute-unit/scripts | jq
```

### Execute a Script

```bash
curl -X POST http://localhost:8080/api/v1/components/compute-unit/scripts/inject-nan/executions \
  -H "Content-Type: application/json" \
  -d '{"execution_type":"now"}' | jq
```

### Check Execution Status

```bash
curl http://localhost:8080/api/v1/components/compute-unit/scripts/inject-nan/executions/<exec_id> | jq
```

### Override Gateway URL

```bash
# Point scripts at a non-default gateway
GATEWAY_URL=http://192.168.1.10:8080 ./inject-nan.sh
```

### Available Scripts

| Script | Description |
|--------|-------------|
| `run-diagnostics` | Check health of all sensors |
| `inject-fault-scenario` | Composite fault injection (all sensors) |
| `inject-drift` | Inject sensor drift on LiDAR |
| `inject-failure` | Inject sensor failure on IMU |
| `inject-nan` | Inject NaN values on LiDAR, IMU, GPS |
| `inject-noise` | Inject high noise on LiDAR and Camera |
| `restore-normal` | Reset all sensors and clear faults |

## Triggers (Condition-Based Alerts)

The gateway supports condition-based triggers that fire when specific events occur, delivering notifications via Server-Sent Events (SSE). This demo creates a fault-monitoring trigger that alerts whenever a new fault is reported.

### Setup

```bash
# Terminal 1: Start the demo
./run-demo.sh

# Terminal 2: Create the fault trigger
./setup-triggers.sh

# Terminal 3: Watch for trigger events (blocking - Ctrl+C to stop)
./watch-triggers.sh

# Terminal 2: Inject a fault - the trigger fires in Terminal 3!
./inject-nan.sh
```

### How It Works

1. `setup-triggers.sh` creates a trigger via `POST /api/v1/components/compute-unit/triggers`:
   - **Resource:** `/api/v1/components/compute-unit/faults` (watches fault collection)
   - **Condition:** `OnChange` (fires on any new or updated fault)
   - **Multishot:** `true` (fires repeatedly, not just once)
   - **Lifetime:** 3600 seconds (auto-expires after 1 hour)
2. `watch-triggers.sh` connects to the SSE event stream at the trigger's `event_source` URL
3. When a fault is injected and detected by the gateway, the trigger fires and an SSE event is delivered

### Manual API Usage

```bash
# Create a trigger
curl -X POST http://localhost:8080/api/v1/components/compute-unit/triggers \
  -H "Content-Type: application/json" \
  -d '{
    "resource": "/api/v1/components/compute-unit/faults",
    "trigger_condition": {"condition_type": "OnChange"},
    "multishot": true,
    "lifetime": 3600
  }' | jq

# List triggers
curl http://localhost:8080/api/v1/components/compute-unit/triggers | jq

# Watch events (replace TRIGGER_ID)
curl -N http://localhost:8080/api/v1/components/compute-unit/triggers/TRIGGER_ID/events

# Delete a trigger
curl -X DELETE http://localhost:8080/api/v1/components/compute-unit/triggers/TRIGGER_ID
```

## API Examples

### Read Sensor Data

```bash
# Get LiDAR scan
curl http://localhost:8080/api/v1/apps/lidar-sim/data/scan | jq '.ranges[:5]'

# Get IMU data
curl http://localhost:8080/api/v1/apps/imu-sim/data/imu | jq '.linear_acceleration'

# Get GPS fix
curl http://localhost:8080/api/v1/apps/gps-sim/data/fix | jq '{lat: .latitude, lon: .longitude}'
```

### View Configurations

```bash
# List all LiDAR configurations
curl http://localhost:8080/api/v1/apps/lidar-sim/configurations | jq

# Get specific parameter
curl http://localhost:8080/api/v1/apps/lidar-sim/configurations/noise_stddev | jq
```

### Inject Faults

```bash
# Increase sensor noise
curl -X PUT http://localhost:8080/api/v1/apps/lidar-sim/configurations/noise_stddev \
  -H "Content-Type: application/json" \
  -d '{"value": 0.5}'

# Cause sensor timeout
curl -X PUT http://localhost:8080/api/v1/apps/lidar-sim/configurations/failure_probability \
  -H "Content-Type: application/json" \
  -d '{"value": 1.0}'

# Inject NaN values
curl -X PUT http://localhost:8080/api/v1/apps/lidar-sim/configurations/inject_nan \
  -H "Content-Type: application/json" \
  -d '{"value": true}'
```

### Check Faults

```bash
# List detected faults
curl http://localhost:8080/api/v1/faults | jq
```

## Configurable Fault Scenarios

| Fault | Description | Parameter |
|-------|-------------|-----------|
| `SENSOR_TIMEOUT` | No messages published | `failure_probability: 1.0` |
| `SENSOR_NAN` | Invalid readings | `inject_nan: true` |
| `HIGH_NOISE` | Degraded accuracy | `noise_stddev: 0.5` |
| `DRIFTING` | Gradual sensor drift | `drift_rate: 0.1` |
| `BLACK_FRAME` | Camera black frames | `inject_black_frames: true` |

## Scripts

| Script | Description |
|--------|-------------|
| `run-demo.sh` | Start Docker services (daemon mode) |
| `stop-demo.sh` | Stop Docker services |
| `check-demo.sh` | Interactive API demonstration and exploration |
| `run-diagnostics.sh` | Check health of all sensors |
| `inject-fault-scenario.sh` | Composite fault injection across all sensors |
| `inject-noise.sh` | Inject high noise fault |
| `inject-failure.sh` | Cause sensor timeout |
| `inject-nan.sh` | Inject NaN values |
| `inject-drift.sh` | Enable sensor drift |
| `restore-normal.sh` | Clear all faults |
| `setup-triggers.sh` | Create OnChange fault trigger |
| `watch-triggers.sh` | Watch trigger events via SSE stream |

> **Note:** All diagnostic scripts (`inject-*.sh`, `restore-normal.sh`, `run-diagnostics.sh`, `inject-fault-scenario.sh`) are also available via the [Scripts API](#scripts-api) - callable as REST endpoints without requiring the host-side scripts.

## Sensor Parameters

### LiDAR (`/sensors/lidar_sim`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `scan_rate` | double | 10.0 | Publishing rate (Hz) |
| `range_min` | double | 0.12 | Minimum range (m) |
| `range_max` | double | 3.5 | Maximum range (m) |
| `noise_stddev` | double | 0.01 | Noise standard deviation (m) |
| `failure_probability` | double | 0.0 | Probability of failure per cycle |
| `inject_nan` | bool | false | Inject NaN values |
| `drift_rate` | double | 0.0 | Range drift rate (m/s) |

### IMU (`/sensors/imu_sim`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rate` | double | 100.0 | Publishing rate (Hz) |
| `accel_noise_stddev` | double | 0.01 | Acceleration noise (m/s²) |
| `gyro_noise_stddev` | double | 0.001 | Gyroscope noise (rad/s) |
| `drift_rate` | double | 0.0 | Orientation drift (rad/s) |

### GPS (`/sensors/gps_sim`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rate` | double | 1.0 | Publishing rate (Hz) |
| `position_noise_stddev` | double | 2.0 | Position noise (m) |
| `altitude_noise_stddev` | double | 5.0 | Altitude noise (m) |
| `drift_rate` | double | 0.0 | Position drift (m/s) |

### Camera (`/sensors/camera_sim`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `rate` | double | 30.0 | Publishing rate (Hz) |
| `width` | int | 640 | Image width (pixels) |
| `height` | int | 480 | Image height (pixels) |
| `noise_level` | double | 0.0 | Fraction of noisy pixels (0-1) |
| `brightness` | int | 128 | Base brightness (0-255) |
| `inject_black_frames` | bool | false | Randomly inject black frames |

## Beacon Mode (Entity Enrichment)

The gateway's beacon plugins let sensor nodes publish extra metadata (display names, process info, topology hints) that enriches the SOVD entity model at runtime - without modifying the manifest.

Three modes are available, controlled by the `BEACON_MODE` environment variable:

| Mode | Plugin | Mechanism | Description |
|------|--------|-----------|-------------|
| `none` | - | - | Default. No beacon plugins. Entities come from manifest + runtime discovery only. |
| `topic` | topic_beacon | Push (ROS 2 topic) | Sensor nodes publish `MedkitDiscoveryHint` messages on `/ros2_medkit/discovery` every 5s. Gateway subscribes and enriches entities. |
| `param` | parameter_beacon | Pull (ROS 2 parameters) | Sensor nodes declare `ros2_medkit.discovery.*` parameters. Gateway polls them every 5s. |

### Usage

```bash
# Docker - set BEACON_MODE before starting
BEACON_MODE=topic docker compose up -d
BEACON_MODE=param docker compose up -d
docker compose up -d   # default: none

# Local (non-Docker)
BEACON_MODE=topic ros2 launch sensor_diagnostics_demo demo.launch.py
```

### Viewing Beacon Data

When a beacon mode is active, each sensor entity gets enriched with extra metadata visible through the API:

```bash
# Topic beacon metadata
curl http://localhost:8080/api/v1/apps/lidar-sim/x-medkit-topic-beacon | jq

# Parameter beacon metadata
curl http://localhost:8080/api/v1/apps/lidar-sim/x-medkit-param-beacon | jq
```

The beacon data includes:
- **entity_id** - Manifest app ID (e.g., `lidar-sim`)
- **display_name** - Human-friendly name (e.g., `LiDAR Simulator`)
- **component_id** - Parent component (e.g., `lidar-unit`)
- **function_ids** - Function membership (e.g., `sensor-monitoring`)
- **process_id** / **hostname** - Process-level diagnostics
- **metadata** - Sensor-specific key-value pairs (sensor_type, data_topic, frame_id)

### How It Works

**Topic beacon** (push): Each sensor node creates a publisher on `/ros2_medkit/discovery` and publishes a `MedkitDiscoveryHint` message every 5 seconds. The gateway's `topic_beacon` plugin subscribes to this topic and merges the hints into the entity model. Hints have a TTL (default 10s) - if a node stops publishing, the data goes stale.

**Parameter beacon** (pull): Each sensor node declares ROS 2 parameters under the `ros2_medkit.discovery.*` prefix. The gateway's `parameter_beacon` plugin polls all nodes for these parameters every 5 seconds. No explicit publishing is needed - the gateway reads the parameters via the ROS 2 parameter service.

Both mechanisms enrich the same entities defined in the manifest. They do not create new entities (the `allow_new_entities` option is disabled). Only one beacon mode should be active at a time - they serve the same purpose via different transport mechanisms.

## Use Cases

1. **CI/CD Testing** - Validate ros2_medkit without heavy simulation
2. **Tutorials** - Simple environment for learning
3. **IoT Sensors** - Same patterns work for non-robot sensors
4. **API Development** - Fast iteration on gateway features

## Comparison with TurtleBot3 Demo

| | Sensor Demo | TurtleBot3 Demo |
|---|-------------|-----------------|
| Docker image | ~500 MB | ~4 GB |
| Startup time | ~5 seconds | ~60 seconds |
| GPU required | No | Recommended |
| CI compatible | Yes | Difficult |
| Focus | Diagnostics | Navigation |

## License

Apache 2.0 - See [LICENSE](../../LICENSE)
