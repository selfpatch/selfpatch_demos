#!/bin/bash
# Restore normal operation across all ECUs
# Resets all fault injection parameters and clears faults
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
if ! curl -sf "${GATEWAY_URL}/api/v1/health" > /dev/null 2>&1; then
    echo "Error: Gateway not reachable at ${GATEWAY_URL}. Is the demo running?"
    exit 1
fi

echo "Restoring normal operation across all ECUs..."

# Perception ECU
execute_script "components" "perception-ecu" "restore-normal" "Restore normal (Perception ECU)"

# Planning ECU
execute_script "components" "planning-ecu" "restore-normal" "Restore normal (Planning ECU)"

# Actuation ECU
execute_script "components" "actuation-ecu" "restore-normal" "Restore normal (Actuation ECU)"

echo ""
echo "All ECUs restored to normal operation"
