#!/bin/bash
# Reset all planning node parameters to defaults
set -eu

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash

ERRORS=0

# Path planner
ros2 param set /planning/path_planner planning_delay_ms 0 || ERRORS=$((ERRORS + 1))
ros2 param set /planning/path_planner failure_probability 0.0 || ERRORS=$((ERRORS + 1))

# Behavior planner
ros2 param set /planning/behavior_planner inject_wrong_direction false || ERRORS=$((ERRORS + 1))
ros2 param set /planning/behavior_planner failure_probability 0.0 || ERRORS=$((ERRORS + 1))

# Task scheduler
ros2 param set /planning/task_scheduler inject_stuck false || ERRORS=$((ERRORS + 1))
ros2 param set /planning/task_scheduler failure_probability 0.0 || ERRORS=$((ERRORS + 1))

if [ $ERRORS -gt 0 ]; then
    echo "{\"status\": \"partial\", \"errors\": $ERRORS}"
    exit 1
fi

# Clear faults
GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"
echo "Clearing faults..."
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true
sleep 2
curl -sf -X DELETE "${API_BASE}/faults" > /dev/null 2>&1 || true

echo '{"status": "restored", "ecu": "planning"}'
