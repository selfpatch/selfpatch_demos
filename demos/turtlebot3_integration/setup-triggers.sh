#!/bin/bash
# Create fault-monitoring trigger for turtlebot3 integration demo
# Alerts on any fault change reported by the anomaly detector - navigation
# and localization faults are reported directly, not via the diagnostic
# bridge.
export ENTITY_TYPE="apps"
export ENTITY_ID="anomaly-detector"
export INJECT_HINT="./inject-localization-failure.sh"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/../../lib/setup-trigger.sh"
