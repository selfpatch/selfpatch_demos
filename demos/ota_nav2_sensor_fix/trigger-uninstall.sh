#!/bin/bash
# Trigger the SOVD uninstall flow: remove broken_lidar_legacy.

set -eu

GATEWAY_URL="${OTA_GATEWAY_URL:-http://localhost:${OTA_GATEWAY_PORT:-8080}}"
API="${GATEWAY_URL}/api/v1"
ID="broken_lidar_legacy_remove"

if ! curl -fsS "${API}/health" >/dev/null 2>&1; then
    echo "Gateway not reachable at ${GATEWAY_URL}. Start it with: ./run-demo.sh"
    exit 1
fi

echo "Uninstall: ${ID}"
# Uninstall has no artifact to fetch but the gateway state machine still
# needs prepare->execute to advance.
echo "  PUT /updates/${ID}/prepare"
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API}/updates/${ID}/prepare" >/dev/null
sleep 2

echo "  PUT /updates/${ID}/execute"
curl -fsS -X PUT -H 'Content-Type: application/json' -d '{}' \
    "${API}/updates/${ID}/execute" >/dev/null
sleep 4

echo ""
echo "Status after execute:"
curl -fsS "${API}/updates/${ID}/status" | (jq . 2>/dev/null || cat)

if docker ps --format '{{.Names}}' | grep -q '^ota_demo_gateway$'; then
    echo ""
    echo "Live processes (broken_lidar_legacy should be gone):"
    docker exec ota_demo_gateway pgrep -af 'broken_lidar_legacy' \
        2>/dev/null | grep -v 'pgrep' | sed 's/^/  /' || echo "  (none - uninstall succeeded)"
fi
