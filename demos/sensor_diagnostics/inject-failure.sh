#!/bin/bash
# Inject sensor failure
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../../lib/scripts-api.sh
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "compute-unit" "inject-failure" "Inject sensor failure"
