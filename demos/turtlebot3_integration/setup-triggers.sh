#!/bin/bash
# Create fault-monitoring trigger for turtlebot3 integration demo
# Alerts on any fault change reported by the anomaly detector
export ENTITY_TYPE="apps"
# Uses ROS node name (underscore) - must match reporting_sources in FaultEvent
export ENTITY_ID="anomaly_detector"
export INJECT_HINT="./inject-nav-failure.sh"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/../../lib/setup-trigger.sh"
