#!/bin/bash
# Inject path planning delay on Planning ECU
# Sets path-planner planning_delay_ms=5000
# Affects: autonomous-navigation
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "planning-ecu" "inject-planning-delay" "Inject planning delay"
