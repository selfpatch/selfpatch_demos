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

# /health answers before the gateway links the sensor nodes, and a node's
# topic reads come back empty until then. A linked sensor that publishes
# nothing (inject-failure.sh) never gets a message, so it holds the wait for
# SAMPLE_WAIT_SEC at most; it is still read while the wait goes on for others.
# The whole wait ends within DATA_WAIT_SEC plus one request.
DATA_WAIT_SEC="${DATA_WAIT_SEC:-30}"
SAMPLE_WAIT_SEC=5
REQUEST_TIMEOUT_SEC=3
SENSOR_APPS=(lidar-sim imu-sim gps-sim)
SENSOR_TOPICS=(sensors/scan sensors/imu sensors/fix)

case "$DATA_WAIT_SEC" in
    '' | *[!0-9]*)
        echo_error "DATA_WAIT_SEC must be a whole number of seconds, got '${DATA_WAIT_SEC}'."
        exit 1
        ;;
esac
# Base 10: bash reads a number with a leading zero as octal.
DATA_WAIT_SEC=$((10#$DATA_WAIT_SEC))

# True when the gateway has linked APP to its node: its data list is not empty.
sensor_linked() {
    curl -sf -m "$REQUEST_TIMEOUT_SEC" "${API_BASE}/apps/$1/data" \
        | jq -e '.items | length > 0' > /dev/null 2>&1
}

# True when the gateway has a message on APP's TOPIC.
sensor_has_data() {
    curl -sf -m "$REQUEST_TIMEOUT_SEC" "${API_BASE}/apps/$1/data/${2//\//%2F}" \
        | jq -e '.data | type == "object" and length > 0' > /dev/null 2>&1
}

wait_start=$(date +%s)
deadline=$((wait_start + DATA_WAIT_SEC))
# Per sensor: when the link was seen, when it was last read, whether a
# message was read, and which waiting line was printed.
linked_at=()
polled_at=()
has_data=()
announced=()
while :; do
    pending=false
    for i in "${!SENSOR_APPS[@]}"; do
        [ -n "${has_data[$i]:-}" ] && continue
        app=${SENSOR_APPS[$i]}
        [ "$(date +%s)" -lt "$deadline" ] || break 2
        if [ -z "${linked_at[$i]:-}" ]; then
            sensor_linked "$app" && linked_at[i]=$(date +%s)
            polled_at[i]=$(date +%s)
            if [ -z "${linked_at[$i]:-}" ]; then
                if [ -z "${announced[$i]:-}" ]; then
                    echo "Waiting for the gateway to link ${app} (max ${DATA_WAIT_SEC}s)..."
                    announced[i]="link"
                fi
                pending=true
                continue
            fi
            [ "$(date +%s)" -lt "$deadline" ] || break 2
        fi
        if sensor_has_data "$app" "${SENSOR_TOPICS[$i]}"; then
            has_data[i]=1
            continue
        fi
        polled_at[i]=$(date +%s)
        # Past its first-message window a sensor no longer holds the wait.
        [ "$(date +%s)" -lt "$((linked_at[i] + SAMPLE_WAIT_SEC))" ] || continue
        if [ "${announced[$i]:-}" != sample ]; then
            echo "Waiting for a first message from ${app} on /${SENSOR_TOPICS[$i]} (max ${SAMPLE_WAIT_SEC}s)..."
            announced[i]="sample"
        fi
        pending=true
    done
    "$pending" || break
    [ "$(date +%s)" -lt "$deadline" ] || break
    sleep 1
done

# The time reported is how long each sensor was read for. The data sections
# below read every sensor again.
missing=false
for i in "${!SENSOR_APPS[@]}"; do
    [ -n "${has_data[$i]:-}" ] && continue
    missing=true
    app=${SENSOR_APPS[$i]}
    if [ -z "${polled_at[$i]:-}" ]; then
        echo "   ${app} was not waited for."
    elif [ -n "${linked_at[$i]:-}" ]; then
        echo "   No message from ${app} on /${SENSOR_TOPICS[$i]} in the $((polled_at[i] - wait_start))s it was waited for; the sensor may have failed."
    else
        echo "   The gateway did not link ${app} in the $((polled_at[i] - wait_start))s it was waited for."
    fi
done
if "$missing"; then
    echo "   Sections 5-7 read each sensor again."
fi

echo_step "1. Checking Gateway Health"
curl -s "${API_BASE}/health" | jq '.'

echo_step "2. Listing All Areas (Namespaces)"
curl -s "${API_BASE}/areas" | jq '.items[] | {id: .id, name: .name, description: .description}'

echo_step "3. Listing All Components"
curl -s "${API_BASE}/components" | jq '.items[] | {id: .id, name: .name, description: .description}'

echo_step "4. Listing All Apps (ROS 2 Nodes)"
curl -s "${API_BASE}/apps" | jq '.items[] | {id: .id, name: .name, component: .["x-medkit"].component_id}'

# Prints FILTER applied to the latest message on APP's TOPIC, or says that
# the gateway has none.
# Usage: show_sensor_data LABEL APP TOPIC FILTER
show_sensor_data() {
    local body
    body=$(curl -s -m 10 "${API_BASE}/apps/$2/data/${3//\//%2F}")
    if echo "$body" | jq -e '.data | type == "object" and length > 0' > /dev/null 2>&1; then
        echo "$body" | jq "$4"
    else
        echo "   No $1 data: the gateway has no message from $2 on /$3."
    fi
}

echo_step "5. Reading LiDAR Data"
echo "Getting latest scan from LiDAR simulator..."
show_sensor_data "LiDAR" lidar-sim sensors/scan '{
  angle_min: .data.angle_min,
  angle_max: .data.angle_max,
  range_min: .data.range_min,
  range_max: .data.range_max,
  sample_ranges: .data.ranges[:5]
}'

echo_step "6. Reading IMU Data"
echo "Getting latest IMU reading..."
show_sensor_data "IMU" imu-sim sensors/imu '{
  linear_acceleration: .data.linear_acceleration,
  angular_velocity: .data.angular_velocity
}'

echo_step "7. Reading GPS Fix"
echo "Getting current GPS position..."
show_sensor_data "GPS" gps-sim sensors/fix '{
  latitude: .data.latitude,
  longitude: .data.longitude,
  altitude: .data.altitude,
  status: .data.status
}'

echo_step "8. Listing LiDAR Configurations"
echo "These parameters can be modified at runtime to inject faults..."
# The list endpoint carries id/name/type only; the value and the ROS type are
# on each parameter's own detail endpoint.
LIDAR_CONFIG_IDS=$(curl -s "${API_BASE}/apps/lidar-sim/configurations" | jq -r '.items[]?.id' 2>/dev/null)
if [ -z "$LIDAR_CONFIG_IDS" ]; then
    echo "   No LiDAR configurations available."
fi
while IFS= read -r cfg_id; do
    [ -n "$cfg_id" ] || continue
    curl -s "${API_BASE}/apps/lidar-sim/configurations/${cfg_id}" \
        | jq '{name: .id, value: .data, type: .["x-medkit"].parameter.type}'
done <<< "$LIDAR_CONFIG_IDS"

echo_step "9. Checking Current Faults"
# A failed read leaves the fault list unknown, not empty.
FAULTS_RESPONSE=$(curl -s -w "\n%{http_code}" "${API_BASE}/faults")
FAULTS_CODE=$(tail -n 1 <<< "$FAULTS_RESPONSE")
FAULTS_JSON=$(sed '$d' <<< "$FAULTS_RESPONSE")
if [ "$FAULTS_CODE" != "200" ] || ! echo "$FAULTS_JSON" | jq -e '.items | type == "array"' > /dev/null 2>&1; then
    echo_error "Could not read faults (HTTP ${FAULTS_CODE}): $(echo "$FAULTS_JSON" | jq -r '.message // empty' 2>/dev/null)"
    echo "   The fault list is unknown. Check that the fault manager is running, then retry."
    exit 1
fi
echo "$FAULTS_JSON" | jq '.'

# If there are faults, demonstrate snapshot / bulk-data endpoints
FAULT_COUNT=$(echo "$FAULTS_JSON" | jq '.items | length')
if [ "$FAULT_COUNT" -gt 0 ]; then
    # The fault collection carries fault_code and reporting_sources (ROS node
    # paths), not an entity id. The owning App is the one whose ROS node is the
    # first reporting source or a path above it, whole segments only: the
    # anomaly detector reports as /processing/anomaly_detector/<sensor>.
    FIRST_FAULT=$(echo "$FAULTS_JSON" | jq -r '.items[0].fault_code')
    REPORTING_SOURCE=$(echo "$FAULTS_JSON" | jq -r '.items[0].reporting_sources[0] // empty')
    FIRST_ENTITY=$(curl -s "${API_BASE}/apps" | jq -r --arg src "$REPORTING_SOURCE" '
        [.items[] | .["x-medkit"].ros2.node as $node
         | select(($node | type) == "string"
                  and ($src == $node or ($src | startswith($node + "/"))))
         | {id, depth: ($node | length)}]
        | max_by(.depth) | .id // empty' 2>/dev/null)

    if [ -z "$FIRST_ENTITY" ]; then
        echo ""
        echo "   Could not map fault ${FIRST_FAULT} to a reporting App (source: ${REPORTING_SOURCE:-none})."
        echo "   Skipping snapshot and bulk-data demonstration."
    else
        echo_step "10. Fault Detail with Environment Data (Snapshots)"
        echo "Fetching fault ${FIRST_FAULT} on apps/${FIRST_ENTITY}..."
        curl -s "${API_BASE}/apps/${FIRST_ENTITY}/faults/${FIRST_FAULT}" | jq '{
          code: .item.code,
          status: .item.status,
          environment_data: {
            extended_data_records: .environment_data.extended_data_records,
            snapshot_count: (.environment_data.snapshots | length)
          }
        }'

        echo_step "11. Bulk-Data Categories (Rosbag Recordings)"
        echo "Checking available bulk-data categories..."
        curl -s "${API_BASE}/apps/${FIRST_ENTITY}/bulk-data" | jq '.'

        echo_step "12. Bulk-Data Descriptors (Rosbag Files)"
        echo "Listing available rosbag recordings..."
        ROSBAGS_JSON=$(curl -s "${API_BASE}/apps/${FIRST_ENTITY}/bulk-data/rosbags")
        if echo "$ROSBAGS_JSON" | jq -e '.items | length > 0' > /dev/null 2>&1; then
            echo "$ROSBAGS_JSON" | jq '.items[] | {
              id: .id,
              name: .name,
              size: .size,
              mimetype: .mimetype,
              "x-medkit": ."x-medkit"
            }'
        else
            echo "   No rosbag recordings listed for apps/${FIRST_ENTITY}."
        fi
    fi
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
echo "   curl ${API_BASE}/faults | jq                                      # List faults"
echo "   curl ${API_BASE}/apps/diagnostic-bridge/faults/<CODE> | jq        # Fault detail + snapshots"
echo "   curl ${API_BASE}/apps/diagnostic-bridge/bulk-data/rosbags | jq    # List rosbag recordings"
echo ""
echo "🌐 Web UI: http://localhost:3000"
echo "🌐 REST API: http://localhost:8080/api/v1/"
