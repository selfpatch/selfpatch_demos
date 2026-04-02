#!/bin/bash
# Inject path planning delay on Planning ECU
# Sets path-planner planning_delay_ms=5000
# Affects: autonomous-navigation
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
if ! curl -sf "${GATEWAY_URL}/api/v1/health" > /dev/null 2>&1; then
    echo "Error: Gateway not reachable at ${GATEWAY_URL}. Is the demo running?"
    exit 1
fi

execute_script "components" "planning-ecu" "inject-planning-delay" "Inject planning delay"
