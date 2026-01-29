#!/bin/bash
# Send a navigation goal to the TurtleBot3 robot via SOVD API
# Usage: ./send-nav-goal.sh [x] [y] [yaw]
#   x   - target x position (default: 2.0)
#   y   - target y position (default: 0.5)
#   yaw - target orientation in radians (default: 0.0)

set -e

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

# Check for required dependencies
if ! command -v bc &> /dev/null; then
    echo "❌ Error: 'bc' command not found. Please install bc (e.g., 'apt-get install bc')"
    exit 1
fi

if ! command -v jq &> /dev/null; then
    echo "❌ Error: 'jq' command not found. Please install jq (e.g., 'apt-get install jq')"
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

# Check gateway is available
echo "Checking gateway..."
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    echo "   Start with: ./run-demo.sh"
    exit 1
fi
echo "✓ Gateway is healthy"

# Calculate quaternion from yaw (rotation around z-axis)
# Full quaternion: x=0, y=0, z=sin(yaw/2), w=cos(yaw/2)
W=$(echo "c($YAW/2)" | bc -l)
Z=$(echo "s($YAW/2)" | bc -l)

echo ""
echo "🤖 Sending navigation goal to TurtleBot3"
echo "   Target: x=$X, y=$Y, yaw=$YAW rad"
echo "   Quaternion: x=0, y=0, z=$Z, w=$W"
echo ""

# Create execution via SOVD API
# bt_navigator exposes NavigateToPose action as an operation
RESPONSE=$(curl -s -X POST "${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions" \
  -H "Content-Type: application/json" \
  -d "{
    \"request\": {
      \"pose\": {
        \"header\": {\"frame_id\": \"map\"},
        \"pose\": {
          \"position\": {\"x\": $X, \"y\": $Y, \"z\": 0.0},
          \"orientation\": {\"x\": 0.0, \"y\": 0.0, \"z\": $Z, \"w\": $W}
        }
      }
    }
  }")

# Check for errors
if echo "$RESPONSE" | jq -e '.error' > /dev/null 2>&1; then
    echo "❌ Navigation goal failed:"
    echo "$RESPONSE" | jq '.error'
    exit 1
fi

# Extract execution ID
EXEC_ID=$(echo "$RESPONSE" | jq -r '.execution_id // .id // empty')

if [ -z "$EXEC_ID" ]; then
    echo "✓ Navigation goal sent (synchronous response)"
    echo "$RESPONSE" | jq '.'
else
    echo "✓ Navigation execution started: $EXEC_ID"
    echo ""
    echo "Check status with:"
    echo "  curl ${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/$EXEC_ID | jq"
    echo ""
    echo "Cancel with:"
    echo "  curl -X DELETE ${API_BASE}/apps/bt-navigator/operations/navigate_to_pose/executions/$EXEC_ID"
fi
