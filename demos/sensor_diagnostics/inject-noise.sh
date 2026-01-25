#!/bin/bash
# Inject high noise fault into sensors

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting HIGH NOISE fault into sensors..."

# LiDAR: increase noise stddev
echo "Setting LiDAR noise_stddev to 0.5 (very noisy)..."
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/noise_stddev" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.5}'

# IMU: increase noise
echo "Setting IMU accel_noise_stddev to 0.5..."
curl -s -X PUT "${API_BASE}/apps/imu_sim/configurations/accel_noise_stddev" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.5}'

# GPS: increase position noise
echo "Setting GPS position_noise_stddev to 50 meters..."
curl -s -X PUT "${API_BASE}/apps/gps_sim/configurations/position_noise_stddev" \
  -H "Content-Type: application/json" \
  -d '{"value": 50.0}'

# Camera: add noise
echo "Setting Camera noise_level to 0.3..."
curl -s -X PUT "${API_BASE}/apps/camera_sim/configurations/noise_level" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.3}'

echo ""
echo "✓ High noise injected! Check faults with: ./run-demo.sh (step 9)"
