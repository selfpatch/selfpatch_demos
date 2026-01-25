#!/bin/bash
# Inject sensor failure (timeout) fault

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting SENSOR FAILURE fault..."

# Set high failure probability - sensors will stop publishing
echo "Setting LiDAR failure_probability to 1.0 (complete failure)..."
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/failure_probability" \
  -H "Content-Type: application/json" \
  -d '{"value": 1.0}'

echo ""
echo "✓ Sensor failure injected! The anomaly detector should report SENSOR_TIMEOUT."
echo "  Check faults with: curl ${API_BASE}/faults | jq"
