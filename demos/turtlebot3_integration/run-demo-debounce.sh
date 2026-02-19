#!/bin/bash
# TurtleBot3 + ros2_medkit Demo Runner - DEBOUNCE MODE
#
# Same as run-demo.sh but uses debounce config:
#   confirmation_threshold: -3 (requires sustained failure)
#   healing_enabled: true (auto-heal on recovery)
#
# Compare with:
#   ./run-demo.sh              → STORM (no debounce, threshold 0)
#   ./run-demo-debounce.sh     → CALM  (debounce, threshold -3)

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🛡️  TurtleBot3 + ros2_medkit — DEBOUNCE MODE"
echo "=============================================="
echo "  confirmation_threshold: -3 (need 3 sustained FAILED events)"
echo "  healing_enabled: true (auto-heal after 3 PASSED events)"
echo ""

# Set TurtleBot3 environment variables
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH:-}:/opt/ros/jazzy/share/turtlebot3_gazebo/models

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    exit 1
fi

# Setup X11 forwarding for GUI (Gazebo)
echo "Setting up X11 forwarding..."
xhost +local:docker 2>/dev/null || {
    echo "   Warning: xhost failed. GUI may not work."
}

# Cleanup function
cleanup() {
    echo ""
    echo "Cleaning up..."
    xhost -local:docker 2>/dev/null || true
    echo "Done!"
}
trap cleanup EXIT

# Parse arguments
HEADLESS_MODE="false"
DETACH_MODE="true"
PROFILE="cpu"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --nvidia)   PROFILE="nvidia" ;;
        --headless) HEADLESS_MODE="true" ;;
        --attached) DETACH_MODE="false" ;;
        *)          echo "Unknown option: $1"; exit 1 ;;
    esac
    shift
done

export HEADLESS=$HEADLESS_MODE

DETACH_FLAG=""
if [[ "$DETACH_MODE" == "true" ]]; then
    DETACH_FLAG="-d"
fi

echo "Building and starting demo (debounce mode)..."
echo ""
echo "🌐 REST API: http://localhost:8080/api/v1/"
echo "🌐 Web UI:   http://localhost:3000/"
echo ""

# Use docker-compose override to mount debounce config
if docker compose version &> /dev/null; then
    docker compose --profile "$PROFILE" \
        -f docker-compose.yml \
        -f docker-compose.debounce.yml \
        build && \
    docker compose --profile "$PROFILE" \
        -f docker-compose.yml \
        -f docker-compose.debounce.yml \
        up ${DETACH_FLAG}
else
    docker-compose --profile "$PROFILE" \
        -f docker-compose.yml \
        -f docker-compose.debounce.yml \
        build && \
    docker-compose --profile "$PROFILE" \
        -f docker-compose.yml \
        -f docker-compose.debounce.yml \
        up ${DETACH_FLAG}
fi

if [[ "$DETACH_MODE" == "true" ]]; then
    echo ""
    echo "✅ Demo started in DEBOUNCE mode!"
    echo ""
    echo "Fire fault storm to see debounce in action:"
    echo "   docker exec turtlebot3_medkit_demo bash -c \\"
    echo "     'source /opt/ros/jazzy/setup.bash && source /root/demo_ws/install/setup.bash && python3 /root/demo_ws/src/turtlebot3_medkit_demo/scripts/fault_storm.py'"
    echo ""
    echo "🛑 To stop: ./stop-demo.sh"
fi
