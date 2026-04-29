#!/bin/bash
# OTA over SOVD - nav2 sensor-fix demo runner.
# Brings up the gateway (with the dev-grade ota_update_plugin baked in) and
# the FastAPI artifact server. The gateway image bundles a full TurtleBot3 +
# Nav2 + headless Gazebo stack and runs foxglove_bridge on :8765, so the
# demo is self-contained: broken_lidar publishes /scan with a phantom
# obstacle that nav2 + a Foxglove 3D panel both react to. The OTA flow
# swaps broken_lidar -> fixed_lidar and the phantom disappears.

set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DETACH_MODE="true"
UPDATE_IMAGES="false"
BUILD_ARGS=""
SKIP_ARTIFACTS="false"

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --attached         Run in foreground (default: daemon mode)"
    echo "  --update           Pull latest images before running"
    echo "  --no-cache         Build Docker images without cache"
    echo "  --skip-artifacts   Skip rebuilding artifacts/catalog.json"
    echo "  -h, --help         Show this help message"
    echo ""
    echo "Environment:"
    echo "  OTA_GATEWAY_PORT           Host port for gateway HTTP API (default: 8080)"
    echo "  OTA_FOXGLOVE_BRIDGE_PORT   Host port for foxglove_bridge WebSocket (default: 8765)"
    echo ""
    echo "Examples:"
    echo "  $0                       # Daemon mode (default)"
    echo "  $0 --attached            # Foreground with logs"
    echo "  OTA_GATEWAY_PORT=8081 $0 # Use a different host port"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --attached) DETACH_MODE="false" ;;
        --update) UPDATE_IMAGES="true" ;;
        --no-cache) BUILD_ARGS="--no-cache" ;;
        --skip-artifacts) SKIP_ARTIFACTS="true" ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
    shift
done

GATEWAY_PORT="${OTA_GATEWAY_PORT:-8080}"
GATEWAY_URL="http://localhost:${GATEWAY_PORT}"

echo "OTA over SOVD - nav2 sensor-fix demo"
echo "===================================="
echo ""

if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    exit 1
fi

if [[ "$SKIP_ARTIFACTS" != "true" ]]; then
    if [[ ! -x "$SCRIPT_DIR/scripts/build_artifacts.sh" ]]; then
        chmod +x "$SCRIPT_DIR/scripts/build_artifacts.sh"
    fi
    echo "[1/3] Building OTA artifacts (catalog.json + tarballs)..."
    "$SCRIPT_DIR/scripts/build_artifacts.sh"
    echo ""
fi

if docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    COMPOSE_CMD="docker-compose"
fi

if [[ "$UPDATE_IMAGES" == "true" ]]; then
    echo "Pulling latest images..."
    ${COMPOSE_CMD} pull
fi

echo "[2/3] Building and starting demo..."
echo "      (First run pulls ros:jazzy and builds the gateway, ~10 minutes)"
echo ""

DETACH_FLAG=""
if [[ "$DETACH_MODE" == "true" ]]; then
    DETACH_FLAG="-d"
fi

# shellcheck disable=SC2086
if ! ${COMPOSE_CMD} build ${BUILD_ARGS}; then
    echo "Docker build failed. Stopping any partially created containers..."
    ${COMPOSE_CMD} down 2>/dev/null || true
    exit 1
fi

# shellcheck disable=SC2086
${COMPOSE_CMD} up ${DETACH_FLAG}

if [[ "$DETACH_MODE" != "true" ]]; then
    exit 0
fi

echo ""
echo "[3/3] Waiting for gateway to come up..."
for _ in 1 2 3 4 5 6 7 8 9 10 11 12; do
    if curl -fsS "${GATEWAY_URL}/api/v1/health" >/dev/null 2>&1; then
        break
    fi
    sleep 2
done

if ! curl -fsS "${GATEWAY_URL}/api/v1/health" >/dev/null 2>&1; then
    echo "Gateway did not respond on ${GATEWAY_URL} - check logs with:"
    echo "  ${COMPOSE_CMD} logs gateway"
    exit 1
fi

echo ""
echo "Demo is up."
echo ""
echo "  Gateway HTTP API:    ${GATEWAY_URL}/api/v1/"
echo "  Foxglove WebSocket:  ws://localhost:${OTA_FOXGLOVE_BRIDGE_PORT:-8765}"
echo "  Update server:       http://localhost:9000/catalog"
echo ""
echo "Registered updates:"
if command -v jq >/dev/null 2>&1; then
    curl -fsS "${GATEWAY_URL}/api/v1/updates" | jq -r '.items[]' | sed 's/^/    /'
else
    curl -fsS "${GATEWAY_URL}/api/v1/updates"
fi
echo ""
echo "Drive the demo:"
echo "  ./check-demo.sh           # show current state"
echo "  ./trigger-update.sh       # update broken_lidar -> fixed_lidar"
echo "  ./trigger-install.sh      # install obstacle_classifier_v2"
echo "  ./trigger-uninstall.sh    # uninstall broken_lidar_legacy"
echo "  ./stop-demo.sh            # tear down"
echo ""
echo "Connect a UI:"
echo "  Web UI (ros2_medkit_web_ui):"
echo "    npm install && npm run dev"
echo "    open http://localhost:5173 -> Connect -> ${GATEWAY_URL}"
echo ""
echo "  Foxglove Studio (recommended for the 3D narrative):"
echo "    Open connection -> Foxglove WebSocket -> ws://localhost:${OTA_FOXGLOVE_BRIDGE_PORT:-8765}"
echo "    Add a 3D panel: TurtleBot3 in the world, /scan cone shows the phantom"
echo "    Install ros2_medkit_foxglove_extension (npm run local-install) for the"
echo "    'ros2_medkit Updates' panel; set baseUrl to ${GATEWAY_URL}/api/v1"
