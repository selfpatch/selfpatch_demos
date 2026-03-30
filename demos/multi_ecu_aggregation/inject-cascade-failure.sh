#!/bin/bash
# Inject cascade failure across all ECUs
# Sets lidar-driver failure_probability=0.9 + motor-controller torque_noise=5.0 + gripper-controller inject_jam=true
# Affects: all functions degraded
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

echo "Injecting cascade failure across all ECUs..."

# Perception ECU - LiDAR sensor failure
execute_script "components" "perception-ecu" "inject-sensor-failure" "Inject LiDAR sensor failure (Perception ECU)"

# Planning ECU - planning delay
execute_script "components" "planning-ecu" "inject-planning-delay" "Inject planning delay (Planning ECU)"

# Actuation ECU - gripper jam
execute_script "components" "actuation-ecu" "inject-gripper-jam" "Inject gripper jam (Actuation ECU)"

echo ""
echo "Cascade failure injected across all ECUs"
echo "Check: curl http://localhost:8080/api/v1/functions | jq"
