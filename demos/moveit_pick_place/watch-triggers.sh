#!/bin/bash
# Watch trigger events for moveit pick-and-place demo
# Connects to SSE stream and prints fault events in real time
export ENTITY_TYPE="apps"
export ENTITY_ID="manipulation_monitor"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/../../lib/watch-trigger.sh" "$@"
