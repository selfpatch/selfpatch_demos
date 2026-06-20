#!/bin/bash
set -euo pipefail

GATEWAY="${GATEWAY:-http://localhost:8080}"

echo "Gateway health:"
curl -fsS "${GATEWAY}/api/v1/health" || echo "  FAIL"

echo ""
echo "Apps:"
curl -fsS "${GATEWAY}/api/v1/apps" | jq -c '.items[] | {id, ros_binding}' || true

echo ""
echo "Active faults (CONFIRMED + PREFAILED):"
curl -fsS -X POST -H "Content-Type: application/json" \
  -d '{"statuses": ["CONFIRMED", "PREFAILED"]}' \
  "${GATEWAY}/api/v1/faults/list" | jq -c '.items[] | {fault_code, severity, status, source: .reporting_sources[0]}' || true
