#!/bin/bash
# Inject gripper jam on Actuation ECU
# Sets gripper-controller inject_jam=true
# Affects: obstacle-avoidance, task-execution
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "actuation-ecu" "inject-gripper-jam" "Inject gripper jam"
