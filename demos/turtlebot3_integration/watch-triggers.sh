#!/bin/bash
# Watch trigger events for turtlebot3 integration demo
# Connects to SSE stream and prints fault events in real time
export ENTITY_TYPE="apps"
export ENTITY_ID="anomaly_detector"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/../../lib/watch-trigger.sh" "$@"
