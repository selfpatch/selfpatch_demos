#!/bin/bash
# Inject collision obstacle via Scripts API
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "moveit-planning" "inject-collision" "Inject collision obstacle"
