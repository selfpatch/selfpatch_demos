#!/bin/bash
# Sensor Diagnostics Demo - Interactive API Demonstration
# Explores ros2_medkit capabilities with simulated sensors

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo_step() {
    echo -e "\n${BLUE}=== $1 ===${NC}\n"
}

echo_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

echo_error() {
    echo -e "${RED}✗ $1${NC}"
}

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         Sensor Diagnostics Demo - API Explorer            ║"
echo "║              (ros2_medkit + Simulated Sensors)             ║"
echo "╚════════════════════════════════════════════════════════════╝"

# Check for jq dependency
if ! command -v jq >/dev/null 2>&1; then
    echo_error "'jq' is required but not installed."
    echo "   Please install jq (e.g., 'sudo apt-get install jq') and retry."
    exit 1
fi

# Check gateway health
echo ""
echo "Checking gateway health..."
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo_error "Gateway not available at ${GATEWAY_URL}"
    echo "   Start with: ./run-demo.sh"
    exit 1
fi
echo_success "Gateway is healthy!"

echo_step "1. Checking Gateway Health"
curl -s "${API_BASE}/health" | jq '.'

echo_step "2. Listing All Areas (Namespaces)"
curl -s "${API_BASE}/areas" | jq '.items[] | {id: .id, name: .name, description: .description}'

echo_step "3. Listing All Components"
curl -s "${API_BASE}/components" | jq '.items[] | {id: .id, name: .name, area: .area}'

echo_step "4. Listing All Apps (ROS 2 Nodes)"
curl -s "${API_BASE}/apps" | jq '.items[] | {id: .id, name: .name, namespace: .namespace}'

echo_step "5. Reading LiDAR Data"
echo "Getting latest scan from LiDAR simulator..."
curl -s "${API_BASE}/apps/lidar-sim/data/scan" | jq '{
  angle_min: .angle_min,
  angle_max: .angle_max,
  range_min: .range_min,
  range_max: .range_max,
  sample_ranges: .ranges[:5]
}'

echo_step "6. Reading IMU Data"
echo "Getting latest IMU reading..."
curl -s "${API_BASE}/apps/imu-sim/data/imu" | jq '{
  linear_acceleration: .linear_acceleration,
  angular_velocity: .angular_velocity
}'

echo_step "7. Reading GPS Fix"
echo "Getting current GPS position..."
curl -s "${API_BASE}/apps/gps-sim/data/fix" | jq '{
  latitude: .latitude,
  longitude: .longitude,
  altitude: .altitude,
  status: .status
}'

echo_step "8. Listing LiDAR Configurations"
echo "These parameters can be modified at runtime to inject faults..."
curl -s "${API_BASE}/apps/lidar-sim/configurations" | jq '.items[] | {name: .name, value: .value, type: .type}'

echo_step "9. Checking Current Faults"
curl -s "${API_BASE}/faults" | jq '.'

# If there are faults, demonstrate snapshot / bulk-data endpoints
FAULT_COUNT=$(curl -s "${API_BASE}/faults" | jq '.items | length')
if [ "$FAULT_COUNT" -gt 0 ]; then
    FIRST_FAULT=$(curl -s "${API_BASE}/faults" | jq -r '.items[0].code')
    FIRST_ENTITY=$(curl -s "${API_BASE}/faults" | jq -r '.items[0].entity_id')

    echo_step "10. Fault Detail with Environment Data (Snapshots)"
    echo "Fetching fault ${FIRST_FAULT} on entity ${FIRST_ENTITY}..."
    curl -s "${API_BASE}/${FIRST_ENTITY}/faults/${FIRST_FAULT}" | jq '{
      code: .item.code,
      status: .item.status,
      environment_data: {
        extended_data_records: .environment_data.extended_data_records,
        snapshot_count: (.environment_data.snapshots | length)
      }
    }'

    echo_step "11. Bulk-Data Categories (Rosbag Recordings)"
    echo "Checking available bulk-data categories..."
    curl -s "${API_BASE}/${FIRST_ENTITY}/bulk-data" | jq '.'

    echo_step "12. Bulk-Data Descriptors (Rosbag Files)"
    echo "Listing available rosbag recordings..."
    curl -s "${API_BASE}/${FIRST_ENTITY}/bulk-data/rosbags" | jq '.items[] | {
      id: .id,
      name: .name,
      size: .size,
      mimetype: .mimetype,
      "x-medkit": ."x-medkit"
    }'
else
    echo ""
    echo "   No active faults. Inject a fault first to see snapshot/bulk-data features:"
    echo "   ./inject-noise.sh && sleep 5 && bash $0"
fi

echo ""
echo_success "API demonstration complete!"
echo ""
echo "🔧 Try injecting faults with these scripts:"
echo "   ./inject-noise.sh        # Increase sensor noise"
echo "   ./inject-failure.sh      # Cause sensor timeouts"
echo "   ./inject-nan.sh          # Inject NaN values"
echo "   ./inject-drift.sh        # Inject sensor drift"
echo "   ./restore-normal.sh      # Restore normal operation"
echo ""
echo "📸 After injecting a fault, check snapshots and rosbags:"
echo "   curl ${API_BASE}/faults | jq                                    # List faults"
echo "   curl ${API_BASE}/components/lidar-unit/faults/<CODE> | jq       # Fault detail + snapshots"
echo "   curl ${API_BASE}/components/lidar-unit/bulk-data/rosbags | jq   # List rosbag recordings"
echo ""
echo "🌐 Web UI: http://localhost:3000"
echo "🌐 REST API: http://localhost:8080/api/v1/"
