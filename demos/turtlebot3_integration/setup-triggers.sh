#!/bin/bash
# Create fault-monitoring trigger for turtlebot3 integration demo
# Alerts on any fault change reported via the diagnostic bridge - the
# anomaly-detector app has no faults of its own, faults arrive from
# /diagnostics through the bridge.
export ENTITY_TYPE="apps"
export ENTITY_ID="diagnostic-bridge"
export INJECT_HINT="./inject-nav-failure.sh"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/../../lib/setup-trigger.sh"
