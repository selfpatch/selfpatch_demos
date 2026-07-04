#!/bin/bash
# Register the remediation update by hand via SOVD POST /updates.
#
# The boot catalog holds only the bad update broken_lidar_3_0_0. The fix is a
# NEW update you publish yourself: its descriptor is a plain JSON file you
# provide - updates/fixed_lidar_3_0_1.json (the id, target component, executable,
# the artifact to fetch, etc.). This just POSTs that file. To do it fully by
# hand, or to register a different update, edit the JSON and POST it directly:
#
#   curl -X POST http://localhost:8080/api/v1/updates \
#        -H 'Content-Type: application/json' \
#        -d @updates/fixed_lidar_3_0_1.json
#
# The artifact named in x_medkit_artifact_url must exist on the update server
# (the demo ships fixed_lidar-3.0.1.tar.gz there); apply-fix.sh then fetches it.

set -eu

GATEWAY_URL="${OTA_GATEWAY_URL:-http://localhost:${OTA_GATEWAY_PORT:-8080}}"
API="${GATEWAY_URL}/api/v1"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
META="${SCRIPT_DIR}/updates/fixed_lidar_3_0_1.json"

if ! curl -fsS "${API}/health" >/dev/null 2>&1; then
    echo "Gateway not reachable at ${GATEWAY_URL}. Start it with: ./run-demo.sh"
    exit 1
fi

echo "Registering the update described in:"
echo "  ${META}"
echo "  -> POST ${API}/updates"
curl -fsS -X POST -H 'Content-Type: application/json' \
    -d @"${META}" "${API}/updates" >/dev/null

echo ""
echo "/updates now offers:"
curl -fsS "${API}/updates" | (jq -r '.items[]' 2>/dev/null || cat)
