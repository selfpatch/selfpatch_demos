#!/bin/bash
# Reset all actuation node parameters to defaults
set -eu

# shellcheck source=/dev/null
source /opt/ros/jazzy/setup.bash
# shellcheck source=/dev/null
source /root/demo_ws/install/setup.bash

ERRORS=0

# Motor controller
ros2 param set /actuation/motor_controller torque_noise 0.01 || ERRORS=$((ERRORS + 1))
ros2 param set /actuation/motor_controller failure_probability 0.0 || ERRORS=$((ERRORS + 1))

# Joint driver
ros2 param set /actuation/joint_driver inject_overheat false || ERRORS=$((ERRORS + 1))
ros2 param set /actuation/joint_driver drift_rate 0.0 || ERRORS=$((ERRORS + 1))
ros2 param set /actuation/joint_driver failure_probability 0.0 || ERRORS=$((ERRORS + 1))

# Gripper controller
ros2 param set /actuation/gripper_controller inject_jam false || ERRORS=$((ERRORS + 1))
ros2 param set /actuation/gripper_controller failure_probability 0.0 || ERRORS=$((ERRORS + 1))

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

echo '{"status": "restored", "ecu": "actuation"}'
