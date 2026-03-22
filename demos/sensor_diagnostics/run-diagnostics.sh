#!/bin/bash
# Run sensor diagnostics
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "compute-unit" "run-diagnostics" "Run sensor diagnostics"
