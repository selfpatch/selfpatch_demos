# Sensor Diagnostics Demo

Lightweight sensor diagnostics demo for **ros2_medkit** - no Gazebo required!

This demo showcases ros2_medkit's data monitoring, configuration management, and fault detection using simulated sensor nodes with configurable fault injection.

## Features

- **Runs anywhere** - No Gazebo, no GPU, works in CI and GitHub Codespaces
- **Fast startup** - Seconds vs minutes compared to TurtleBot3 demo
- **Focus on diagnostics** - Pure ros2_medkit features without robot complexity
- **Configurable faults** - Runtime fault injection via REST API
- **Dual fault reporting** - Demonstrates both legacy (diagnostics) and modern (direct) paths

## Quick Start

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
| `inject-noise.sh` | Inject high noise fault |
| `inject-failure.sh` | Cause sensor timeout |
| `inject-nan.sh` | Inject NaN values |
| `inject-drift.sh` | Enable sensor drift |
| `restore-normal.sh` | Clear all faults |

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
