#!/bin/bash
# TurtleBot3 + ros2_medkit Demo Runner

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🤖 TurtleBot3 + ros2_medkit Demo"
echo "=================================="

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    exit 1
fi

# Setup X11 forwarding for GUI (Gazebo)
echo "🖥️  Setting up X11 forwarding..."
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

# Select compose file
if [[ "$1" == "--nvidia" ]]; then
    echo "Using NVIDIA GPU acceleration"
    COMPOSE_FILE="docker-compose.nvidia.yml"
else
    COMPOSE_FILE="docker-compose.yml"
fi

# Build and run
echo "   Building and starting demo..."
echo "   (First run takes ~5-10 min, downloading ~4GB image)"
echo ""

if docker compose version &> /dev/null; then
    docker compose -f "$COMPOSE_FILE" up --build
else
    docker-compose -f "$COMPOSE_FILE" up --build
fi
