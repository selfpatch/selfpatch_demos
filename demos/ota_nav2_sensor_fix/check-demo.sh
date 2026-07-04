#!/bin/bash
# Show the live state of the OTA demo at a glance: which lidar build
# scan_sensor_node is running, the applied updates + their statuses, and
# the current Nav2 faults on bt-navigator + controller-server (so a latched
# ACTION_NAVIGATE_TO_POSE_ABORTED / LOG_* fault and the bad update that
# caused it are both visible in one shot).

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

GATEWAY_RUNNING="false"
if docker ps --format '{{.Names}}' | grep -q '^ota_demo_gateway$'; then
    GATEWAY_RUNNING="true"
fi

echo "Gateway:  ${GATEWAY_URL}"
echo "Health:   $(curl -fsS "${API}/health" | head -c 200)"
echo ""

echo "Scan sensor (scan_sensor_node):"
if [[ "$GATEWAY_RUNNING" == "true" ]]; then
    SCAN_PROC=$(docker exec ota_demo_gateway pgrep -af 'broken_lidar_node|fixed_lidar_node' \
        2>/dev/null | grep -v 'pgrep' || true)
    if echo "$SCAN_PROC" | grep -q 'broken_lidar_node'; then
        echo "  broken_lidar_node running - REGRESSED build applied (root cause)"
    elif echo "$SCAN_PROC" | grep -q 'fixed_lidar_node'; then
        echo "  fixed_lidar_node running - known-good build"
    else
        echo "  (no scan sensor process found)"
    fi
else
    echo "  ota_demo_gateway container not running"
fi
echo ""

echo "Applied updates (GET /updates, GET /updates/{id}/status):"
if [[ "$JQ_AVAILABLE" == "true" ]]; then
    for id in $(curl -fsS "${API}/updates" | jq -r '.items[]'); do
        status=$(curl -fsS "${API}/updates/${id}/status" 2>/dev/null || echo '{"status":"<no status>"}')
        echo "  ${id}:  $(echo "$status" | jq -c '{status, progress}')"
    done
else
    curl -fsS "${API}/updates"
fi
echo ""

echo "Current Nav2 faults (GET /apps/bt-navigator/faults, /apps/controller-server/faults):"
for entity in "apps/bt-navigator" "apps/controller-server"; do
    echo "  ${entity}:"
    FAULTS_JSON=$(curl -fsS "${API}/${entity}/faults" 2>/dev/null || echo '{"items":[]}')
    if [[ "$JQ_AVAILABLE" == "true" ]]; then
        FAULT_COUNT=$(echo "$FAULTS_JSON" | jq '.items | length')
        if [[ "$FAULT_COUNT" -eq 0 ]]; then
            echo "    (none - clean)"
        else
            echo "$FAULTS_JSON" | jq -r '.items[] | "    \(.fault_code): \(.status)"'
        fi
    else
        echo "    $FAULTS_JSON"
    fi
done
echo ""

echo "Plugin-managed processes inside gateway container:"
if [[ "$GATEWAY_RUNNING" == "true" ]]; then
    docker exec ota_demo_gateway pgrep -af \
        'broken_lidar_node|fixed_lidar_node' \
        2>/dev/null | grep -v 'pgrep' | sed 's/^/  /' || echo "  (none)"
else
    echo "  ota_demo_gateway container not running"
fi
