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

# Cleanup function (undo the X11 grant only if we actually set it up)
X11_FORWARDING="false"
cleanup() {
    echo ""
    echo "Cleaning up..."
    if [[ "$X11_FORWARDING" == "true" ]]; then
        xhost -local:docker 2>/dev/null || true
    fi
    echo "Done!"
}
trap cleanup EXIT

# Parse arguments
HEADLESS_MODE="${HEADLESS:-false}"
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

# Auto-enable headless when there is no display to render the Gazebo GUI.
# macOS Docker Desktop has no X server, and headless Linux hosts have no DISPLAY;
# in both cases the Gazebo window cannot open and would abort the whole launch.
# An explicit --headless (or HEADLESS=true) always wins.
if [[ "$HEADLESS_MODE" != "true" && -z "${DISPLAY:-}" ]]; then
    if [[ "$(uname -s)" == "Darwin" ]]; then
        echo "ℹ️  macOS detected with no X display: running HEADLESS."
        echo "    Docker Desktop on macOS cannot open the Gazebo 3D window."
    else
        echo "ℹ️  No DISPLAY detected: running HEADLESS (no Gazebo 3D window)."
    fi
    echo "    The simulation and ros2_medkit still run normally:"
    echo "      REST API -> http://localhost:8080/api/v1/"
    echo "      Web UI   -> http://localhost:3000/"
    HEADLESS_MODE="true"
fi

# Set up X11 forwarding only when a GUI will actually be shown
if [[ "$HEADLESS_MODE" != "true" ]]; then
    echo "Setting up X11 forwarding..."
    if xhost +local:docker 2>/dev/null; then
        X11_FORWARDING="true"
    else
        echo "   Warning: xhost failed. GUI may not work."
    fi
fi

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
    if [[ "$PROFILE" == "nvidia" ]]; then
        CONTAINER="turtlebot3_medkit_demo_nvidia"
    else
        CONTAINER="turtlebot3_medkit_demo"
    fi
    echo ""
    echo "✅ Demo started in DEBOUNCE mode!"
    echo ""
    echo "Fire fault storm to see debounce in action:"
    echo "   docker exec $CONTAINER bash -c \\"
    echo "     'source /opt/ros/jazzy/setup.bash && source /root/demo_ws/install/setup.bash && python3 /root/demo_ws/src/turtlebot3_medkit_demo/scripts/fault_storm.py'"
    echo ""
    echo "🛑 To stop: ./stop-demo.sh"
fi
