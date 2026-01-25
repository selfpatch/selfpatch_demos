#!/bin/bash
# Sensor Diagnostics Demo - Interactive Demo Script
# Run this script to see ros2_medkit in action with simulated sensors

set -e

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo_step() {
    echo -e "\n${BLUE}=== $1 ===${NC}\n"
}

echo_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

echo_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

echo_error() {
    echo -e "${RED}✗ $1${NC}"
}

wait_for_gateway() {
    echo "Waiting for gateway to be ready..."
    for _ in {1..30}; do
        if curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
            echo_success "Gateway is ready!"
            return 0
        fi
        sleep 1
    done
    echo_error "Gateway not available at ${GATEWAY_URL}"
    exit 1
}

# Main demo flow
echo "╔════════════════════════════════════════════════════════════╗"
echo "║         Sensor Diagnostics Demo with ros2_medkit          ║"
echo "║              (Lightweight - No Gazebo Required)           ║"
echo "╚════════════════════════════════════════════════════════════╝"

echo_step "1. Checking Gateway Health"
wait_for_gateway
curl -s "${API_BASE}/health" | jq '.'

echo_step "2. Listing All Areas (Namespaces)"
curl -s "${API_BASE}/areas" | jq '.'

echo_step "3. Listing All Components"
curl -s "${API_BASE}/components" | jq '.items[] | {id: .id, name: .name, area: .area}'

echo_step "4. Listing All Apps (ROS 2 Nodes)"
curl -s "${API_BASE}/apps" | jq '.items[] | {id: .id, name: .name, namespace: .namespace}'

echo_step "5. Reading LiDAR Data"
echo "Getting latest scan from LiDAR simulator..."
curl -s "${API_BASE}/apps/lidar_sim/data/scan" | jq '{
  angle_min: .angle_min,
  angle_max: .angle_max,
  range_min: .range_min,
  range_max: .range_max,
  sample_ranges: .ranges[:5]
}'

echo_step "6. Reading IMU Data"
echo "Getting latest IMU reading..."
curl -s "${API_BASE}/apps/imu_sim/data/imu" | jq '{
  linear_acceleration: .linear_acceleration,
  angular_velocity: .angular_velocity
}'

echo_step "7. Reading GPS Fix"
echo "Getting current GPS position..."
curl -s "${API_BASE}/apps/gps_sim/data/fix" | jq '{
  latitude: .latitude,
  longitude: .longitude,
  altitude: .altitude,
  status: .status
}'

echo_step "8. Listing LiDAR Configurations"
echo "These parameters can be modified at runtime to inject faults..."
curl -s "${API_BASE}/apps/lidar_sim/configurations" | jq '.items[] | {name: .name, value: .value, type: .type}'

echo_step "9. Checking Current Faults"
curl -s "${API_BASE}/faults" | jq '.'

echo ""
echo_success "Demo complete!"
echo ""
echo "Try injecting faults with these scripts:"
echo "  ./inject-noise.sh     - Increase sensor noise"
echo "  ./inject-failure.sh   - Cause sensor timeouts"
echo "  ./inject-nan.sh       - Inject NaN values"
echo "  ./restore-normal.sh   - Restore normal operation"
echo ""
echo "Or use the Web UI at http://localhost:3000"
