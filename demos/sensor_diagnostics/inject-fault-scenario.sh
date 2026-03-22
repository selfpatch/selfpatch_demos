#!/bin/bash
# Inject composite fault scenario
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "compute-unit" "inject-fault-scenario" "Inject composite fault scenario"
