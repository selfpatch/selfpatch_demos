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
# The ota_update_server image self-builds its catalog + tarballs in-image (no
# host `./artifacts` mount is used), so a host-side artifact rebuild is not
# part of the reproducible path. Default to skipping it; --build-artifacts is
# an opt-in for maintainers who are iterating on scripts/build_artifacts.sh
# and want the host-built output for local inspection.
SKIP_ARTIFACTS="true"

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --attached         Run in foreground (default: daemon mode)"
    echo "  --update           Pull latest images before running"
    echo "  --no-cache         Build Docker images without cache"
    echo "  --build-artifacts  Rebuild artifacts/catalog.json on the host before starting"
    echo "                     (maintainer opt-in; requires host ROS + ros2_medkit_msgs)"
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
        --build-artifacts) SKIP_ARTIFACTS="false" ;;
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

# Drive-readiness gate. The gz_ros2_control hardware interface + the
# joint_state_broadcaster / diff_drive_controller spawners race the sim's
# cold start; when they lose, diff_drive publishes no odometry, the
# odom->base_footprint TF never appears, and Nav2 aborts every goal instantly
# (the robot never moves). That race is more likely when the host CPU is
# loaded. Rather than hand off a demo that cannot drive, wait for real
# odometry; if it does not come up, restart the sim and try again (bounded),
# so `run-demo.sh` only reports "up" once the robot is genuinely drive-ready.
GW_CONTAINER="${OTA_DEMO_GATEWAY_CONTAINER:-ota_demo_gateway}"
drive_ready() {
    docker exec "${GW_CONTAINER}" bash -lc \
        'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; \
         timeout 6 ros2 run tf2_ros tf2_echo odom base_footprint > /tmp/_navcheck 2>&1; \
         grep -qE "Translation|At time" /tmp/_navcheck' >/dev/null 2>&1
}

echo ""
echo "[3b/3] Waiting for the robot to become drive-ready (odometry + TF)..."
DRIVE_READY=false
for boot_try in 1 2 3; do
    for _ in $(seq 1 13); do
        if drive_ready; then DRIVE_READY=true; break; fi
        sleep 10
    done
    if [[ "$DRIVE_READY" == "true" ]]; then
        echo "      Drive-ready: odometry and odom->base_footprint TF are live."
        break
    fi
    if [[ "$boot_try" -lt 3 ]]; then
        echo "      No odometry after ~130s (gz_ros2_control cold-start race)."
        echo "      Restarting the sim (attempt $((boot_try + 1))/3)..."
        ${COMPOSE_CMD} restart gateway >/dev/null 2>&1 || true
        for _ in $(seq 1 40); do
            curl -fsS "${GATEWAY_URL}/api/v1/health" >/dev/null 2>&1 && break
            sleep 3
        done
    fi
done

if [[ "$DRIVE_READY" != "true" ]]; then
    echo ""
    echo "WARNING: the robot's odometry/TF did not come up after 3 sim starts."
    echo "  This is a gz_ros2_control cold-start race, more likely under host CPU load."
    echo "  Free host CPU (close heavy apps; on WSL, 'wsl --shutdown') and re-run ./run-demo.sh."
    echo "  Check manually:"
    echo "    docker exec ${GW_CONTAINER} bash -lc 'source /opt/ros/jazzy/setup.bash && ros2 run tf2_ros tf2_echo odom base_footprint'"
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
echo "  ./publish-fix.sh          # register fixed_lidar_3_0_1 (SOVD POST /updates) - not in the boot catalog"
echo "  ./apply-fix.sh            # apply the published fix: broken_lidar -> fixed_lidar_3_0_1"
echo "  ./trigger-bad-update.sh   # re-arm broken_lidar (root cause) for a rerun"
echo "  ./clear-fault.sh          # operator clear of the latched bt-navigator/controller-server faults"
echo "  ./send-goal.sh            # send a nav goal (mission start / resume)"
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
