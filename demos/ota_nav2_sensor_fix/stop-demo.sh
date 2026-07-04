#!/bin/bash
# Stop the OTA over SOVD - nav2 sensor-fix demo.

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

REMOVE_VOLUMES=""
REMOVE_IMAGES=""

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -v, --volumes    Remove named volumes"
    echo "  --images         Remove built images"
    echo "  -h, --help       Show this help message"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -v|--volumes) REMOVE_VOLUMES="-v" ;;
        --images) REMOVE_IMAGES="--rmi local" ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
    shift
done

if docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

# shellcheck disable=SC2086
${COMPOSE_CMD} down ${REMOVE_VOLUMES} ${REMOVE_IMAGES}
echo ""
echo "Demo stopped."
