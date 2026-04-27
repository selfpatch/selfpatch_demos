#!/bin/bash
# Show the live state of the OTA demo: registered updates, per-update
# status, and the demo node processes the plugin manages inside the
# gateway container.

set -eu

GATEWAY_URL="${OTA_GATEWAY_URL:-http://localhost:${OTA_GATEWAY_PORT:-8080}}"
API="${GATEWAY_URL}/api/v1"

if ! command -v curl >/dev/null 2>&1; then
    echo "curl is required"
    exit 1
fi

if ! curl -fsS "${API}/health" >/dev/null 2>&1; then
    echo "Gateway not reachable at ${GATEWAY_URL}. Start it with: ./run-demo.sh"
    exit 1
fi

JQ_AVAILABLE="false"
if command -v jq >/dev/null 2>&1; then
    JQ_AVAILABLE="true"
fi

echo "Gateway:  ${GATEWAY_URL}"
echo "Health:   $(curl -fsS "${API}/health" | head -c 200)"
echo ""

echo "Registered updates (GET /updates):"
if [[ "$JQ_AVAILABLE" == "true" ]]; then
    curl -fsS "${API}/updates" | jq -r '.items[]' | sed 's/^/  /'
else
    curl -fsS "${API}/updates"
fi
echo ""

echo "Per-update status (GET /updates/{id}/status):"
if [[ "$JQ_AVAILABLE" == "true" ]]; then
    for id in $(curl -fsS "${API}/updates" | jq -r '.items[]'); do
        status=$(curl -fsS "${API}/updates/${id}/status" 2>/dev/null || echo '{"status":"<no status>"}')
        echo "  ${id}:  $(echo "$status" | jq -c '{status, progress}')"
    done
else
    echo "  (install jq for detail)"
fi
echo ""

echo "Plugin-managed processes inside gateway container:"
if docker ps --format '{{.Names}}' | grep -q '^ota_demo_gateway$'; then
    docker exec ota_demo_gateway pgrep -af \
        'broken_lidar_node|fixed_lidar_node|broken_lidar_legacy|obstacle_classifier' \
        2>/dev/null | grep -v 'pgrep' | sed 's/^/  /' || echo "  (none)"
else
    echo "  ota_demo_gateway container not running"
fi
