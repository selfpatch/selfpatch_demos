#!/bin/bash
# Sensor Diagnostics Demo Runner
# Starts Docker services with ros2_medkit and simulated sensors

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Parse arguments
DETACH_MODE="true"
UPDATE_IMAGES="false"
BUILD_ARGS=""

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --attached      Run in foreground (default: daemon mode)"
    echo "  --update        Pull latest images before running"
    echo "  --no-cache      Build Docker images without cache"
    echo "  -h, --help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                   # Daemon mode (default)"
    echo "  $0 --attached        # Foreground with logs"
    echo "  $0 --update          # Pull and run latest version"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --attached)
            echo "Running in foreground mode"
            DETACH_MODE="false"
            ;;
        --update)
            echo "Will pull latest images"
            UPDATE_IMAGES="true"
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

echo "🔬 Sensor Diagnostics Demo with ros2_medkit"
echo "============================================"
echo "   (Lightweight - No Gazebo Required)"
echo ""

# Check for Docker
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    exit 1
fi

echo "Run mode: $([ "$DETACH_MODE" = "true" ] && echo "daemon (background)" || echo "attached (foreground)")"
echo ""

# Pull images if --update flag is set
if [[ "$UPDATE_IMAGES" == "true" ]]; then
    echo "📥 Pulling latest images..."
    if docker compose version &> /dev/null; then
        docker compose pull
    else
        docker-compose pull
    fi
    echo ""
fi

# Build and start services
echo "🚀 Building and starting demo..."
echo "   (First run may take a few minutes)"
echo ""
echo "🌐 REST API will be available at: http://localhost:8080/api/v1/"
echo "🌐 Web UI will be available at: http://localhost:3000/"
echo ""

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
if ! ${COMPOSE_CMD} build ${BUILD_ARGS}; then
    echo ""
    echo "❌ Docker build failed! Stopping any partially created containers..."
    ${COMPOSE_CMD} down 2>/dev/null || true
    exit 1
fi

# shellcheck disable=SC2086
${COMPOSE_CMD} up ${DETACH_FLAG}

if [[ "$DETACH_MODE" == "true" ]]; then
    echo ""
    echo "✅ Demo started in background!"
    echo ""
    echo "📊 To view logs:"
    echo "   docker compose logs -f"
    echo ""
    echo "🔬 Explore the API:"
    echo "   ./check-demo.sh          # Interactive API demonstration"
    echo ""
    echo "🔧 Inject faults:"
    echo "   ./inject-noise.sh        # Increase sensor noise"
    echo "   ./inject-failure.sh      # Cause sensor timeouts"
    echo "   ./inject-nan.sh          # Inject NaN values"
    echo "   ./inject-drift.sh        # Inject sensor drift"
    echo "   ./restore-normal.sh      # Restore normal operation"
    echo ""
    echo "🌐 Web UI: http://localhost:3000"
    echo "🌐 REST API: http://localhost:8080/api/v1/"
    echo ""
    echo "🛑 To stop: ./stop-demo.sh"
fi
