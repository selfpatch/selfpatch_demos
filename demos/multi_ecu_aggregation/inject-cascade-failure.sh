#!/bin/bash
# Inject cascade failure across all ECUs
# Injects: lidar-driver failure_probability=0.8 + path-planner planning_delay_ms=5000 + gripper-controller inject_jam=true
# Affects: all functions degraded
set -u
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

echo "Injecting cascade failure across all ECUs..."

ERRORS=0

# Perception ECU - LiDAR sensor failure
execute_script "components" "perception-ecu" "inject-sensor-failure" "Inject LiDAR sensor failure (Perception ECU)" || ERRORS=$((ERRORS + 1))

# Planning ECU - planning delay
execute_script "components" "planning-ecu" "inject-planning-delay" "Inject planning delay (Planning ECU)" || ERRORS=$((ERRORS + 1))

# Actuation ECU - gripper jam
execute_script "components" "actuation-ecu" "inject-gripper-jam" "Inject gripper jam (Actuation ECU)" || ERRORS=$((ERRORS + 1))

echo ""
if [ $ERRORS -gt 0 ]; then
    echo "Warning: $ERRORS ECU(s) failed injection"
    exit 1
fi
echo "Cascade failure injected across all ECUs"
echo "Check: curl http://localhost:8080/api/v1/functions | jq"
