# Sensor Diagnostics Demo

Lightweight sensor diagnostics demo for **ros2_medkit** - no Gazebo required!

This demo showcases ros2_medkit's data monitoring, configuration management, and fault detection using simulated sensor nodes with configurable fault injection.

## Features

- **Runs anywhere** - No Gazebo, no GPU, works in CI and GitHub Codespaces
- **Fast startup** - Seconds vs minutes compared to TurtleBot3 demo
- **Focus on diagnostics** - Pure ros2_medkit features without robot complexity
- **Configurable faults** - Runtime fault injection via REST API

## Quick Start

### Using Docker (Recommended)

```bash
# Start the demo
docker compose up

# In another terminal, open the Web UI
# Navigate to http://localhost:3000

# Or run the demo script
./run-demo.sh
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
│   ├── lidar_sim              # 2D LiDAR (LaserScan)
│   ├── imu_sim                # 9-DOF IMU (Imu)
│   ├── gps_sim                # GPS receiver (NavSatFix)
│   └── camera_sim             # RGB camera (Image, CameraInfo)
├── /processing                 # Data processing
│   └── anomaly_detector       # Fault detection node
└── /diagnostics                # Monitoring
    └── ros2_medkit_gateway    # REST API gateway
```

## API Examples

### Read Sensor Data

```bash
# Get LiDAR scan
curl http://localhost:8080/api/v1/apps/lidar_sim/data/scan | jq '.ranges[:5]'

# Get IMU data
curl http://localhost:8080/api/v1/apps/imu_sim/data/imu | jq '.linear_acceleration'

# Get GPS fix
curl http://localhost:8080/api/v1/apps/gps_sim/data/fix | jq '{lat: .latitude, lon: .longitude}'
```

### View Configurations

```bash
# List all LiDAR configurations
curl http://localhost:8080/api/v1/apps/lidar_sim/configurations | jq

# Get specific parameter
curl http://localhost:8080/api/v1/apps/lidar_sim/configurations/noise_stddev | jq
```

### Inject Faults

```bash
# Increase sensor noise
curl -X PUT http://localhost:8080/api/v1/apps/lidar_sim/configurations/noise_stddev \
  -H "Content-Type: application/json" \
  -d '{"value": 0.5}'

# Cause sensor timeout
curl -X PUT http://localhost:8080/api/v1/apps/lidar_sim/configurations/failure_probability \
  -H "Content-Type: application/json" \
  -d '{"value": 1.0}'

# Inject NaN values
curl -X PUT http://localhost:8080/api/v1/apps/lidar_sim/configurations/inject_nan \
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
| `DRIFT_DETECTED` | Gradual sensor drift | `drift_rate: 0.1` |
| `BLACK_FRAME` | Camera black frames | `inject_black_frames: true` |

## Scripts

| Script | Description |
|--------|-------------|
| `run-demo.sh` | Interactive demo walkthrough |
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
