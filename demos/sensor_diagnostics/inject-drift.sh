#!/bin/bash
# Inject sensor drift fault

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting DRIFT fault into sensors..."

# Set drift rates
echo "Setting LiDAR drift_rate to 0.1 m/s..."
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/drift_rate" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.1}'

echo "Setting IMU drift_rate to 0.01 rad/s..."
curl -s -X PUT "${API_BASE}/apps/imu_sim/configurations/drift_rate" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.01}'

echo "Setting GPS drift_rate to 1.0 m/s..."
curl -s -X PUT "${API_BASE}/apps/gps_sim/configurations/drift_rate" \
  -H "Content-Type: application/json" \
  -d '{"value": 1.0}'

echo ""
echo "✓ Drift enabled! Sensor readings will gradually shift over time."
echo "  Watch the drift accumulate with: ./run-demo.sh"
