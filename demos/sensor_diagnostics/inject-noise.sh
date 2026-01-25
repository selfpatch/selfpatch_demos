#!/bin/bash
# Inject high noise fault - demonstrates LEGACY fault reporting path
# LiDAR/Camera → DiagnosticArray → /diagnostics → diagnostic_bridge → FaultManager

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting HIGH NOISE fault (Legacy path: LiDAR/Camera → diagnostic_bridge)..."
echo ""

# LiDAR: increase noise stddev (uses legacy diagnostics path)
echo "[LEGACY PATH] Setting LiDAR noise_stddev to 0.5 (very noisy)..."
echo "  Fault path: lidar_sim → /diagnostics topic → diagnostic_bridge → FaultManager"
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/noise_stddev" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.5}'
echo ""

# Camera: add noise (uses legacy diagnostics path)
echo "[LEGACY PATH] Setting Camera noise_level to 0.3..."
echo "  Fault path: camera_sim → /diagnostics topic → diagnostic_bridge → FaultManager"
curl -s -X PUT "${API_BASE}/apps/camera_sim/configurations/noise_level" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.3}'

echo ""
echo "✓ High noise injected on LiDAR and Camera!"
echo ""
echo "Fault codes expected (auto-generated from diagnostic names):"
echo "  - LIDAR_SIM (HIGH_NOISE status)"
echo "  - CAMERA_SIM (HIGH_NOISE status)"
echo ""
echo "Check faults with: curl ${API_BASE}/faults | jq"
