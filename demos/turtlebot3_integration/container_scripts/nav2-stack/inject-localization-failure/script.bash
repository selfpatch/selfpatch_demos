#!/bin/bash
# Inject localization failure: reinitialize AMCL then send a navigation goal under high uncertainty
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "Reinitializing AMCL global localization (scatters particles)..."
curl -s -X POST "${API_BASE}/apps/amcl/operations/reinitialize_global_localization/executions" \
    -H "Content-Type: application/json" \
    -d '{}'

echo ""
echo "Waiting for particles to scatter..."
sleep 2

echo "Sending navigation goal with high localization uncertainty..."
curl -s -X POST "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" \
    -H "Content-Type: application/json" \
    -d '{
    "goal": {
      "pose": {
        "header": {"frame_id": "map"},
        "pose": {
          "position": {"x": 2.0, "y": 0.0, "z": 0.0},
          "orientation": {"w": 1.0}
        }
      }
    }
  }'

echo ""
echo "Localization failure injected."
echo "AMCL has been reinitialized - localization uncertainty is high."
echo "Monitor faults at: ${API_BASE}/faults"
exit 0
