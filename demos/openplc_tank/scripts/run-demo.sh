#!/usr/bin/env bash
# Run the OpenPLC Tank Demo
set -eu

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DIR="$(dirname "$SCRIPT_DIR")"
cd "$DEMO_DIR"

# Defaults
DETACH_MODE=true
UPDATE=false
NO_CACHE=false

usage() {
  echo "Usage: $0 [OPTIONS]"
  echo ""
  echo "Options:"
  echo "  --attached    Run in foreground (default: detached)"
  echo "  --update      Pull latest images before starting"
  echo "  --no-cache    Build without Docker cache"
  echo "  -h, --help    Show this help"
}

while [[ $# -gt 0 ]]; do
  case $1 in
    --attached) DETACH_MODE=false; shift ;;
    --update)   UPDATE=true; shift ;;
    --no-cache) NO_CACHE=true; shift ;;
    -h|--help)  usage; exit 0 ;;
    *)          echo "Unknown option: $1"; usage; exit 1 ;;
  esac
done

# Detect compose command
if docker compose version &> /dev/null; then
  COMPOSE_CMD="docker compose"
else
  COMPOSE_CMD="docker-compose"
fi

echo "=== OpenPLC Tank Demo ==="
echo "Components:"
echo "  - OpenPLC v4 Runtime (OPC-UA :4840)"
echo "  - ros2_medkit Gateway + OPC-UA Plugin (REST :8080)"
echo "  - SOVD Web UI (:3000)"
echo ""

# Optional: pull latest web UI
if [[ "$UPDATE" == "true" ]]; then
  echo "Pulling latest images..."
  $COMPOSE_CMD pull sovd-web-ui
fi

# Build
BUILD_ARGS=""
if [[ "$NO_CACHE" == "true" ]]; then
  BUILD_ARGS="--no-cache"
fi

echo "Building containers..."
$COMPOSE_CMD build $BUILD_ARGS || {
  echo "Build failed!"
  exit 1
}

# Run
DETACH_FLAG=""
if [[ "$DETACH_MODE" == "true" ]]; then
  DETACH_FLAG="-d"
fi

echo "Starting demo..."
$COMPOSE_CMD up $DETACH_FLAG

if [[ "$DETACH_MODE" == "true" ]]; then
  echo ""
  echo "Demo started in background."
  echo ""
  echo "Next steps:"
  echo "  - Web UI:     http://localhost:3000"
  echo "  - REST API:   http://localhost:8080/api/v1"
  echo "  - OPC-UA:     opc.tcp://localhost:4840"
  echo ""
  echo "  - Check:      curl http://localhost:8080/api/v1/health"
  echo "  - PLC data:   curl http://localhost:8080/api/v1/apps/tank_process/x-plc-data"
  echo "  - Inject:     ./scripts/inject-fault.sh"
  echo "  - Restore:    ./scripts/restore-normal.sh"
  echo "  - Stop:       ./scripts/stop-demo.sh"
fi
