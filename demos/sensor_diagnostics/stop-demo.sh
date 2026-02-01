#!/bin/bash
# Stop Sensor Diagnostics Demo

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "🛑 Stopping Sensor Diagnostics Demo"
echo "====================================="

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    exit 1
fi

# Parse arguments
REMOVE_VOLUMES=""
REMOVE_IMAGES=""

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -v, --volumes    Remove named volumes"
    echo "  --images         Remove images"
    echo "  -h, --help       Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0               # Stop containers"
    echo "  $0 --volumes     # Stop and remove volumes"
    echo "  $0 --images      # Stop and remove images"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -v|--volumes)
            echo "Will remove named volumes"
            REMOVE_VOLUMES="-v"
            ;;
        --images)
            echo "Will remove images"
            REMOVE_IMAGES="--rmi all"
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

# Stop containers
echo "Stopping containers..."
if docker compose version &> /dev/null; then
    # shellcheck disable=SC2086
    docker compose down ${REMOVE_VOLUMES} ${REMOVE_IMAGES}
else
    # shellcheck disable=SC2086
    docker-compose down ${REMOVE_VOLUMES} ${REMOVE_IMAGES}
fi

echo ""
echo "✅ Demo stopped successfully!"
