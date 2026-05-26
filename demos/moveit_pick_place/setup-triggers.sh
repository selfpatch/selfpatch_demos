#!/bin/bash
# Create fault-monitoring trigger for moveit pick-and-place demo
# Alerts on any fault change reported by the manipulation monitor
export ENTITY_TYPE="apps"
# Gateway normalises entity IDs with hyphens (matches the registered apps id).
export ENTITY_ID="manipulation-monitor"
export INJECT_HINT="./inject-planning-failure.sh"
# shellcheck disable=SC1091
source "$(cd "$(dirname "$0")" && pwd)/../../lib/setup-trigger.sh"
