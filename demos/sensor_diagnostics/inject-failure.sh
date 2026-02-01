#!/bin/bash
# Inject sensor failure (timeout) fault - demonstrates MODERN fault reporting path
# IMU sensor → anomaly-detector → FaultManager (via ReportFault service)

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting SENSOR FAILURE fault (Modern path: IMU → anomaly-detector)..."

# Set high failure probability - IMU will stop publishing
echo "Setting IMU failure_probability to 1.0 (complete failure)..."
curl -s -X PUT "${API_BASE}/apps/imu-sim/configurations/failure_probability" \
  -H "Content-Type: application/json" \
  -d '{"value": 1.0}'

echo ""
echo "✓ IMU failure injected!"
echo "  Fault reporting path: imu-sim → anomaly-detector → /fault_manager/report_fault"
echo "  The anomaly detector should report SENSOR_TIMEOUT fault directly to FaultManager."
echo "  Check faults with: curl ${API_BASE}/faults | jq"
