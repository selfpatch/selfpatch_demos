#!/bin/bash
# Operator clear of the latched Nav2 faults after the fix is applied.
# bt-navigator's ACTION_NAVIGATE_TO_POSE_ABORTED is a stable, bridge-generated
# code, so it is cleared by code. controller-server's LOG_CONTROLLER_SERVER_*
# code is content-derived (the hash can change between runs), so it is
# cleared with a clear-all on the entity instead of a hardcoded code. Both
# faults are latched (no self-heal); this is the deliberate operator
# acknowledge step.
set -eu
API="${OTA_GATEWAY_URL:-http://localhost:${OTA_GATEWAY_PORT:-8080}}/api/v1"
NAV_ENTITY="apps/bt-navigator"
NAV_CODE="ACTION_NAVIGATE_TO_POSE_ABORTED"
CONTROLLER_ENTITY="apps/controller-server"

echo "DELETE /${NAV_ENTITY}/faults/${NAV_CODE}"
curl -fsS -X DELETE "${API}/${NAV_ENTITY}/faults/${NAV_CODE}" -o /dev/null -w '  HTTP %{http_code}\n'

echo "DELETE /${CONTROLLER_ENTITY}/faults (clear-all - the LOG_* code is content-hashed)"
curl -fsS -X DELETE "${API}/${CONTROLLER_ENTITY}/faults" -o /dev/null -w '  HTTP %{http_code}\n'

echo "Remaining faults on ${NAV_ENTITY}:"
curl -fsS "${API}/${NAV_ENTITY}/faults" | (jq -r '.items[].fault_code' 2>/dev/null || cat) | sed 's/^/  /'
echo "Remaining faults on ${CONTROLLER_ENTITY}:"
curl -fsS "${API}/${CONTROLLER_ENTITY}/faults" | (jq -r '.items[].fault_code' 2>/dev/null || cat) | sed 's/^/  /'
