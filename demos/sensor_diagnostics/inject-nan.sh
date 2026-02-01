#!/bin/bash
# Inject NaN values fault - demonstrates BOTH fault reporting paths

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting NaN VALUES fault (demonstrates both fault reporting paths)..."
echo ""

# LEGACY PATH: LiDAR publishes DiagnosticArray → diagnostic_bridge → FaultManager
echo "[LEGACY PATH] Enabling LiDAR inject_nan..."
echo "  Fault path: lidar-sim → /diagnostics topic → diagnostic-bridge → FaultManager"
curl -s -X PUT "${API_BASE}/apps/lidar-sim/configurations/inject_nan" \
  -H "Content-Type: application/json" \
  -d '{"value": true}'
echo ""

# MODERN PATH: IMU/GPS → anomaly-detector → FaultManager (direct service call)
echo "[MODERN PATH] Enabling IMU inject_nan..."
echo "  Fault path: imu-sim → anomaly-detector → /fault_manager/report_fault"
curl -s -X PUT "${API_BASE}/apps/imu-sim/configurations/inject_nan" \
  -H "Content-Type: application/json" \
  -d '{"value": true}'
echo ""

echo "[MODERN PATH] Enabling GPS inject_nan..."
echo "  Fault path: gps-sim → anomaly-detector → /fault_manager/report_fault"
curl -s -X PUT "${API_BASE}/apps/gps-sim/configurations/inject_nan" \
  -H "Content-Type: application/json" \
  -d '{"value": true}'

echo ""
echo "✓ NaN injection enabled on multiple sensors!"
echo ""
echo "Fault codes expected:"
echo "  - LIDAR_SIM (from diagnostic-bridge, auto-generated from diagnostic name)"
echo "  - SENSOR_NAN (from anomaly-detector)"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
