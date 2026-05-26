#!/bin/bash
# Arm self-test via Scripts API
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "moveit-planning" "arm-self-test" "Arm self-test"
