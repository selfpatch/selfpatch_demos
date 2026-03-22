#!/bin/bash
# Check nav2 stack health: verify key apps are responding via gateway
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

APPS=("amcl" "bt-navigator" "controller-server" "planner-server")
ERRORS=0

echo "Checking nav2 stack health..."
echo ""

for app in "${APPS[@]}"; do
    echo "Checking ${app}..."
    if curl -sf "${API_BASE}/apps/${app}" > /dev/null 2>&1; then
        echo "  OK: ${app} responding"
    else
        echo "  FAIL: ${app} not responding"
        ERRORS=$((ERRORS + 1))
    fi
done

echo ""
FAULT_COUNT=$(curl -sf "${API_BASE}/faults" | jq '.items | length' 2>/dev/null || echo "?")
echo "Active faults: ${FAULT_COUNT}"

if [ "${ERRORS}" -eq 0 ]; then
    echo ""
    echo "All nav2 apps healthy."
    exit 0
else
    echo ""
    echo "${ERRORS} app(s) not responding."
    exit 1
fi
