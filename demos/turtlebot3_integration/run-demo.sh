#!/bin/bash
# TurtleBot3 + ros2_medkit Demo Runner with Nav2 Navigation

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🤖 TurtleBot3 + ros2_medkit + Nav2 Demo"
echo "========================================="

# Set TurtleBot3 environment variables
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH:-}:/opt/ros/jazzy/share/turtlebot3_gazebo/models

echo "TurtleBot3 Model: $TURTLEBOT3_MODEL"

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    exit 1
fi

# Setup X11 forwarding for GUI (Gazebo, RViz)
echo "Setting up X11 forwarding..."
xhost +local:docker 2>/dev/null || {
    echo "   Warning: xhost failed. GUI may not work."
    echo "   Install with: sudo apt install x11-xserver-utils"
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
COMPOSE_ARGS=""
BUILD_ARGS=""

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --nvidia     Use NVIDIA GPU acceleration"
    echo "  --no-cache   Build Docker images without cache"
    echo "  -h, --help   Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                      # CPU-only mode"
    echo "  $0 --nvidia             # With GPU acceleration"
    echo "  $0 --no-cache           # Rebuild without cache"
    echo "  $0 --nvidia --no-cache  # Both options"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --nvidia)
            echo "Using NVIDIA GPU acceleration"
            COMPOSE_ARGS="--profile nvidia"
            ;;
        --no-cache)
            echo "Building without cache"
            BUILD_ARGS="--no-cache"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
    shift
done

if [[ -z "$COMPOSE_ARGS" ]]; then
    echo "Using CPU-only mode (use --nvidia flag for GPU acceleration)"
fi

# Build and run
echo "   Building and starting demo..."
echo "   (First run takes ~5-10 min, downloading ~4GB image)"
echo ""
echo "📍 After launch, use the following to send navigation goals:"
echo "   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \\"
echo "     \"{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}}\""
echo ""
echo "🌐 REST API available at: http://localhost:8080/api/v1/"
echo "🌐 Web UI available at: http://localhost:3000/"
echo ""

if docker compose version &> /dev/null; then
    # shellcheck disable=SC2086
    docker compose ${COMPOSE_ARGS} build ${BUILD_ARGS} && \
    docker compose ${COMPOSE_ARGS} up
else
    # shellcheck disable=SC2086
    docker-compose ${COMPOSE_ARGS} build ${BUILD_ARGS} && \
    docker-compose ${COMPOSE_ARGS} up
fi
