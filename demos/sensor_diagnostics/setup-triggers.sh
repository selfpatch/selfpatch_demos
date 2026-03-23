#!/bin/bash
# Create fault-monitoring trigger for sensor diagnostics demo
# Alerts on any new fault reported via the diagnostic bridge
export ENTITY_TYPE="apps"
# Uses ROS node name (underscore) - must match reporting_sources in FaultEvent
export ENTITY_ID="diagnostic_bridge"
export INJECT_HINT="./inject-nan.sh"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/../../lib/setup-trigger.sh"
