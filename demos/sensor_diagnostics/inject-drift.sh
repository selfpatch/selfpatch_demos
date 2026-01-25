#!/bin/bash
# Inject sensor drift fault - demonstrates LEGACY fault reporting path
# LiDAR drift → DiagnosticArray → /diagnostics → diagnostic_bridge → FaultManager

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Injecting DRIFT fault (Legacy path: LiDAR → diagnostic_bridge)..."
echo ""

# LiDAR drift: uses legacy diagnostics path
echo "[LEGACY PATH] Setting LiDAR drift_rate to 0.1 m/s..."
echo "  Fault path: lidar_sim → /diagnostics topic → diagnostic_bridge → FaultManager"
curl -s -X PUT "${API_BASE}/apps/lidar_sim/configurations/drift_rate" \
  -H "Content-Type: application/json" \
  -d '{"value": 0.1}'

echo ""
echo "✓ Drift enabled! LiDAR readings will gradually shift over time."
echo ""
echo "Fault codes expected (auto-generated from diagnostic name):"
echo "  - LIDAR_SIM (DRIFTING status, WARN severity)"
echo ""
echo "Watch the drift accumulate with: curl ${GATEWAY_URL}/api/v1/apps/lidar_sim/data/scan | jq '.ranges[:5]'"
echo "Check faults with: curl ${GATEWAY_URL}/api/v1/faults | jq"
