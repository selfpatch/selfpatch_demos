#!/bin/bash
# Send a navigation goal to the TurtleBot3 robot
# Usage: ./send-nav-goal.sh [x] [y] [yaw]
#   x   - target x position (default: 2.0)
#   y   - target y position (default: 0.5)
#   yaw - target orientation in radians (default: 0.0)

set -e

CONTAINER_NAME="turtlebot3_medkit_demo"

# Check for required dependencies
if ! command -v bc &> /dev/null; then
    echo "❌ Error: 'bc' command not found. Please install bc (e.g., 'apt-get install bc')"
    exit 1
fi

# Default goal position
X=${1:-2.0}
Y=${2:-0.5}
YAW=${3:-0.0}

# Validate that inputs are numeric (prevents command injection)
validate_numeric() {
    local value="$1"
    local name="$2"
    if ! [[ "$value" =~ ^-?[0-9]*\.?[0-9]+$ ]]; then
        echo "❌ Error: '$name' must be a numeric value, got: '$value'"
        exit 1
    fi
}

validate_numeric "$X" "x"
validate_numeric "$Y" "y"
validate_numeric "$YAW" "yaw"

# Calculate quaternion from yaw (rotation around z-axis)
# Full quaternion: x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)
W=$(echo "c($YAW/2)" | bc -l)
Z=$(echo "s($YAW/2)" | bc -l)

echo "🤖 Sending navigation goal to TurtleBot3"
echo "   Target: x=$X, y=$Y, yaw=$YAW rad"
echo "   Quaternion: x=0, y=0, z=$Z, w=$W"
echo ""

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ Container '$CONTAINER_NAME' is not running!"
    echo "   Start with: ./run-demo.sh"
    exit 1
fi

# Send the navigation goal
# Using validated numeric values - safe to interpolate after validation
docker exec -it "$CONTAINER_NAME" bash -c "
source /opt/ros/jazzy/setup.bash && \
source /root/demo_ws/install/setup.bash && \
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: $Z, w: $W}}}}\"
"
