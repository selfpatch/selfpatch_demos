#!/bin/bash
# Trigger the SOVD update flow: replace broken_lidar with fixed_lidar.
# Uses spec endpoints PUT /updates/{id}/prepare then PUT /updates/{id}/execute.

set -eu

GATEWAY_URL="${OTA_GATEWAY_URL:-http://localhost:${OTA_GATEWAY_PORT:-8080}}"
API="${GATEWAY_URL}/api/v1"
ID="fixed_lidar_2_1_0"

if ! curl -fsS "${API}/health" >/dev/null 2>&1; then
    echo "Gateway not reachable at ${GATEWAY_URL}. Start it with: ./run-demo.sh"
    exit 1
fi

echo "Update: ${ID}"
echo "  PUT /updates/${ID}/prepare"
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API}/updates/${ID}/prepare" >/dev/null
sleep 3

echo "  PUT /updates/${ID}/execute"
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API}/updates/${ID}/execute" >/dev/null
sleep 5

echo ""
echo "Status after execute:"
curl -fsS "${API}/updates/${ID}/status" | (jq . 2>/dev/null || cat)

if docker ps --format '{{.Names}}' | grep -q '^ota_demo_gateway$'; then
    echo ""
    echo "Live processes:"
    docker exec ota_demo_gateway pgrep -af 'broken_lidar_node|fixed_lidar_node' \
        2>/dev/null | grep -v 'pgrep' | sed 's/^/  /' || true
fi
