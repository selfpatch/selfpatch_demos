#!/bin/bash
# Inject LiDAR sensor failure on Perception ECU
# Affects: autonomous-navigation, obstacle-avoidance
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "perception-ecu" "inject-sensor-failure" "Inject LiDAR sensor failure"
