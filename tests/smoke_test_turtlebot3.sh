#!/bin/bash
# Smoke tests for turtlebot3_integration demo
# Runs from the host against the containerized gateway on localhost:8080
#
# Tests: health, entity discovery (areas, components, apps from manifest)
# No fault injection - Gazebo-based demo is too complex for reliable CI fault testing
#
# Usage: ./tests/smoke_test_turtlebot3.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

# --- Wait for gateway startup ---

# Turtlebot3 needs Gazebo + Nav2 - allow extra startup time
wait_for_gateway 120

# Wait for runtime node linking
wait_for_runtime_linking "/apps/medkit-gateway/data" 90

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

test_entity_discovery "areas" robot navigation diagnostics bridge
test_entity_discovery "components" turtlebot3-base lidar-sensor nav2-stack gateway fault-manager diagnostic-bridge-unit
test_entity_discovery "apps" turtlebot3-node robot-state-publisher amcl bt-navigator controller-server planner-server velocity-smoother medkit-gateway medkit-fault-manager diagnostic-bridge anomaly-detector

# --- Summary ---

print_summary
