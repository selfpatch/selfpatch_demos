#!/bin/bash
# Inject NaN values fault into sensors

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting NaN VALUES fault into sensors..."

# Enable NaN injection on sensors
echo "Enabling LiDAR inject_nan..."
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/inject_nan" \
  -H "Content-Type: application/json" \
  -d '{"value": true}'

echo "Enabling IMU inject_nan..."
curl -s -X PUT "${API_BASE}/apps/imu_sim/configurations/inject_nan" \
  -H "Content-Type: application/json" \
  -d '{"value": true}'

echo "Enabling GPS inject_nan..."
curl -s -X PUT "${API_BASE}/apps/gps_sim/configurations/inject_nan" \
  -H "Content-Type: application/json" \
  -d '{"value": true}'

echo ""
echo "✓ NaN injection enabled! Anomaly detector should report SENSOR_NAN faults."
echo "  Check faults with: curl ${API_BASE}/faults | jq"
