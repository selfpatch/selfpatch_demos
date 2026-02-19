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
HEADLESS_MODE="false"
UPDATE_IMAGES="false"
DETACH_MODE="true"

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --nvidia     Use NVIDIA GPU acceleration"
    echo "  --no-cache   Build Docker images without cache"
    echo "  --headless   Run without Gazebo GUI (default: GUI enabled)"
    echo "  --update     Pull latest images before running"
    echo "  --attached   Run in foreground (default: daemon mode)"
    echo "  -h, --help   Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                      # Daemon mode (default)"
    echo "  $0 --attached           # Foreground with logs"
    echo "  $0 --headless           # Headless mode (no GUI)"
    echo "  $0 --nvidia             # GPU acceleration + GUI"
    echo "  $0 --no-cache           # Rebuild without cache"
    echo "  $0 --update             # Pull and run latest version"
    echo ""
    echo "Environment variables:"
    echo "  HEADLESS=true|false    Control GUI mode (default: false)"
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
        --headless)
            echo "Running in headless mode (no GUI)"
            HEADLESS_MODE="true"
            ;;
        --update)
            echo "Will pull latest images"
            UPDATE_IMAGES="true"
            ;;
        --attached)
            echo "Running in foreground mode"
            DETACH_MODE="false"
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
    COMPOSE_ARGS="--profile cpu"
fi

# Export HEADLESS mode for docker-compose
export HEADLESS=$HEADLESS_MODE
echo "Gazebo mode: $([ "$HEADLESS_MODE" = "true" ] && echo "headless (no GUI)" || echo "GUI enabled")"
echo "Run mode: $([ "$DETACH_MODE" = "true" ] && echo "daemon (background)" || echo "attached (foreground)")"

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

# Pull images if --update flag is set
if [[ "$UPDATE_IMAGES" == "true" ]]; then
    echo "📥 Pulling latest images..."
    if docker compose version &> /dev/null; then
        # shellcheck disable=SC2086
        docker compose ${COMPOSE_ARGS} pull
    else
        # shellcheck disable=SC2086
        docker-compose ${COMPOSE_ARGS} pull
    fi
    echo ""
fi

# Set detach flag
DETACH_FLAG=""
if [[ "$DETACH_MODE" == "true" ]]; then
    DETACH_FLAG="-d"
fi

if docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

# shellcheck disable=SC2086
if ! ${COMPOSE_CMD} ${COMPOSE_ARGS} build ${BUILD_ARGS}; then
    echo ""
    echo "❌ Docker build failed! Stopping any partially created containers..."
    # shellcheck disable=SC2086
    ${COMPOSE_CMD} ${COMPOSE_ARGS} down 2>/dev/null || true
    exit 1
fi

# shellcheck disable=SC2086
${COMPOSE_CMD} ${COMPOSE_ARGS} up ${DETACH_FLAG}

if [[ "$DETACH_MODE" == "true" ]]; then
    echo ""
    echo "✅ Demo started in background!"
    echo ""
    echo "📊 To view logs:"
    echo "   docker compose --profile cpu logs -f      # CPU version"
    echo "   docker compose --profile nvidia logs -f   # NVIDIA version"
    echo ""
    echo "🔧 To interact with ROS 2:"
    echo "   docker exec -it turtlebot3_medkit_demo bash       # CPU"
    echo "   docker exec -it turtlebot3_medkit_demo_nvidia bash # NVIDIA"
    echo ""
    echo "🛑 To stop: ./stop-demo.sh"
fi
