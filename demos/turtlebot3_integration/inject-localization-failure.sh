#!/bin/bash
# Inject localization failure via Scripts API
set -eu
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=../../lib/scripts-api.sh
source "${SCRIPT_DIR}/../../lib/scripts-api.sh"

execute_script "components" "nav2-stack" "inject-localization-failure" "Inject localization failure"
