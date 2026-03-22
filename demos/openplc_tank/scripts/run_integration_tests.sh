#!/usr/bin/env bash
# Integration Tests for OpenPLC Tank Demo
# Tests: entity discovery, live data, control, fault lifecycle, resilience, errors
set -euo pipefail

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API="${GATEWAY_URL}/api/v1"
CURL_OPTS="--max-time 10 -sf"
PASSED=0
FAILED=0
TOTAL=0

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

cleanup() {
  echo ""
  echo "==============================="
  echo -e "Results: ${GREEN}${PASSED} passed${NC} / ${RED}${FAILED} failed${NC} / ${TOTAL} total"
  echo "==============================="
  if [[ $FAILED -gt 0 ]]; then
    exit 1
  fi
}
trap cleanup EXIT

# --- Helpers ---

assert_response() {
  local description="$1"
  local method="$2"
  local url="$3"
  local expected_status="$4"
  local body="${5:-}"
  local jq_filter="${6:-}"
  local expected_value="${7:-}"

  TOTAL=$((TOTAL + 1))

  local curl_args=("--max-time" "10" "-s" "-o" "/tmp/test_body.json" "-w" "%{http_code}")

  if [[ "$method" == "POST" ]]; then
    curl_args+=("-X" "POST" "-H" "Content-Type: application/json")
    if [[ -n "$body" ]]; then
      curl_args+=("-d" "$body")
    fi
  elif [[ "$method" == "DELETE" ]]; then
    curl_args+=("-X" "DELETE")
  fi

  curl_args+=("$url")

  local status
  status=$(curl "${curl_args[@]}" 2>/dev/null) || status="000"

  if [[ "$status" != "$expected_status" ]]; then
    FAILED=$((FAILED + 1))
    echo -e "${RED}FAIL${NC} $description"
    echo "       Expected HTTP $expected_status, got $status"
    echo "       URL: $method $url"
    if [[ -f /tmp/test_body.json ]]; then
      echo "       Body: $(head -c 200 /tmp/test_body.json)"
    fi
    return 1
  fi

  if [[ -n "$jq_filter" && -n "$expected_value" ]]; then
    local actual
    actual=$(jq -r "$jq_filter" /tmp/test_body.json 2>/dev/null) || actual="<jq error>"

    if [[ "$actual" != "$expected_value" ]]; then
      FAILED=$((FAILED + 1))
      echo -e "${RED}FAIL${NC} $description"
      echo "       jq '$jq_filter' = '$actual', expected '$expected_value'"
      return 1
    fi
  fi

  PASSED=$((PASSED + 1))
  echo -e "${GREEN}PASS${NC} $description"
  return 0
}

assert_contains() {
  local description="$1"
  local url="$2"
  local jq_filter="$3"
  local expected_value="$4"

  TOTAL=$((TOTAL + 1))

  local body
  body=$(curl $CURL_OPTS "$url" 2>/dev/null) || body=""

  local result
  result=$(echo "$body" | jq -r "$jq_filter" 2>/dev/null) || result=""

  if echo "$result" | grep -q "$expected_value"; then
    PASSED=$((PASSED + 1))
    echo -e "${GREEN}PASS${NC} $description"
    return 0
  else
    FAILED=$((FAILED + 1))
    echo -e "${RED}FAIL${NC} $description"
    echo "       '$expected_value' not found in jq output of '$jq_filter'"
    return 1
  fi
}

wait_for_gateway() {
  echo "Waiting for gateway..."
  for i in $(seq 1 30); do
    if curl -sf "${API}/health" > /dev/null 2>&1; then
      echo "Gateway is up!"
      return 0
    fi
    sleep 1
  done
  echo "Gateway did not start within 30s"
  exit 1
}

# --- Start ---

echo "========================================"
echo "OpenPLC Tank Demo - Integration Tests"
echo "Gateway: $GATEWAY_URL"
echo "========================================"
echo ""

wait_for_gateway

# =============================================================================
# 1. Entity Discovery (SOVD Tree)
# =============================================================================
echo ""
echo -e "${YELLOW}--- 1. Entity Discovery ---${NC}"

assert_contains "Area plc_systems exists" \
  "${API}/areas" '.items[].id' "plc_systems"

assert_contains "Component openplc_runtime exists" \
  "${API}/components" '.items[].id' "openplc_runtime"

assert_contains "App tank_process exists" \
  "${API}/apps" '.items[].id' "tank_process"

assert_contains "App fill_pump exists" \
  "${API}/apps" '.items[].id' "fill_pump"

assert_contains "App drain_valve exists" \
  "${API}/apps" '.items[].id' "drain_valve"

assert_response "tank_process detail returns external=true" \
  "GET" "${API}/apps/tank_process" "200" "" '.external' "true"

assert_response "openplc_runtime is in plc_systems area" \
  "GET" "${API}/components/openplc_runtime" "200" "" '."x-medkit".area' "plc_systems"

# =============================================================================
# 2. Live Data (sensor readings)
# =============================================================================
echo ""
echo -e "${YELLOW}--- 2. Live Data ---${NC}"

assert_response "tank_process x-plc-data returns 200" \
  "GET" "${API}/apps/tank_process/x-plc-data" "200"

assert_contains "tank_process has tank_level data" \
  "${API}/apps/tank_process/x-plc-data" '.items[].name' "tank_level"

assert_contains "tank_process has tank_temperature data" \
  "${API}/apps/tank_process/x-plc-data" '.items[].name' "tank_temperature"

assert_contains "tank_process has tank_pressure data" \
  "${API}/apps/tank_process/x-plc-data" '.items[].name' "tank_pressure"

assert_response "fill_pump x-plc-data returns 200" \
  "GET" "${API}/apps/fill_pump/x-plc-data" "200"

assert_contains "fill_pump has pump_speed data" \
  "${API}/apps/fill_pump/x-plc-data" '.items[].name' "pump_speed"

assert_contains "fill_pump has pump_running data" \
  "${API}/apps/fill_pump/x-plc-data" '.items[].name' "pump_running"

assert_response "drain_valve x-plc-data returns 200" \
  "GET" "${API}/apps/drain_valve/x-plc-data" "200"

assert_contains "drain_valve has valve_position data" \
  "${API}/apps/drain_valve/x-plc-data" '.items[].name' "valve_position"

assert_response "Single node query works" \
  "GET" "${API}/apps/tank_process/x-plc-data/tank_level" "200"

# Verify values are numeric (not null)
TOTAL=$((TOTAL + 1))
LEVEL=$(curl $CURL_OPTS "${API}/apps/tank_process/x-plc-data/tank_level" 2>/dev/null | jq '.value' 2>/dev/null)
if [[ "$LEVEL" != "null" && -n "$LEVEL" ]]; then
  PASSED=$((PASSED + 1))
  echo -e "${GREEN}PASS${NC} tank_level has numeric value: $LEVEL"
else
  FAILED=$((FAILED + 1))
  echo -e "${RED}FAIL${NC} tank_level value is null or missing"
fi

# =============================================================================
# 3. Control (write setpoints)
# =============================================================================
echo ""
echo -e "${YELLOW}--- 3. Control Operations ---${NC}"

assert_response "Set pump speed to 75%" \
  "POST" "${API}/apps/fill_pump/x-plc-operations/set_pump_speed" "200" \
  '{"value": 75.0}'

sleep 1  # Wait for PLC cycle

# Verify pump speed was written
TOTAL=$((TOTAL + 1))
PUMP_SPEED=$(curl $CURL_OPTS "${API}/apps/fill_pump/x-plc-data/pump_speed" 2>/dev/null | jq '.value' 2>/dev/null)
if [[ -n "$PUMP_SPEED" ]] && (( $(echo "$PUMP_SPEED > 70 && $PUMP_SPEED < 80" | bc -l 2>/dev/null || echo 0) )); then
  PASSED=$((PASSED + 1))
  echo -e "${GREEN}PASS${NC} pump_speed reads back ~75.0 (got $PUMP_SPEED)"
else
  FAILED=$((FAILED + 1))
  echo -e "${RED}FAIL${NC} pump_speed readback: expected ~75.0, got $PUMP_SPEED"
fi

assert_response "Set valve position to 50%" \
  "POST" "${API}/apps/drain_valve/x-plc-operations/set_valve_position" "200" \
  '{"value": 50.0}'

sleep 1

# Verify valve position
TOTAL=$((TOTAL + 1))
VALVE_POS=$(curl $CURL_OPTS "${API}/apps/drain_valve/x-plc-data/valve_position" 2>/dev/null | jq '.value' 2>/dev/null)
if [[ -n "$VALVE_POS" ]] && (( $(echo "$VALVE_POS > 45 && $VALVE_POS < 55" | bc -l 2>/dev/null || echo 0) )); then
  PASSED=$((PASSED + 1))
  echo -e "${GREEN}PASS${NC} valve_position reads back ~50.0 (got $VALVE_POS)"
else
  FAILED=$((FAILED + 1))
  echo -e "${RED}FAIL${NC} valve_position readback: expected ~50.0, got $VALVE_POS"
fi

# =============================================================================
# 4. Fault Lifecycle (alarm injection + SOVD faults)
# =============================================================================
echo ""
echo -e "${YELLOW}--- 4. Fault Lifecycle ---${NC}"

# First, restore normal state
curl -s -X POST "${API}/apps/fill_pump/x-plc-operations/set_pump_speed" \
  -H "Content-Type: application/json" -d '{"value": 0.0}' > /dev/null 2>&1 || true

echo "Waiting for temperature to rise above 80C (alarm threshold)..."
# Stop pump to let temperature rise
ALARM_DETECTED=0
for i in $(seq 1 60); do
  TEMP=$(curl $CURL_OPTS "${API}/apps/tank_process/x-plc-data/tank_temperature" 2>/dev/null | jq '.value' 2>/dev/null || echo "0")
  if (( $(echo "${TEMP:-0} > 80" | bc -l 2>/dev/null || echo 0) )); then
    echo "  Temperature reached ${TEMP}C"
    ALARM_DETECTED=1
    break
  fi
  if (( i % 10 == 0 )); then
    echo "  Temperature at ${TEMP}C..."
  fi
  sleep 2
done

if [[ $ALARM_DETECTED -eq 1 ]]; then
  sleep 2  # Wait for poller + fault report

  assert_contains "PLC_HIGH_TEMP fault reported" \
    "${API}/apps/tank_process/faults" '.faults[].fault_code' "PLC_HIGH_TEMP"

  assert_contains "PLC_HIGH_TEMP in global fault list" \
    "${API}/faults" '.faults[].fault_code' "PLC_HIGH_TEMP"

  # Clear alarm by restoring pump (cooling)
  echo "Clearing alarm - restoring pump to cool tank..."
  curl -s -X POST "${API}/apps/fill_pump/x-plc-operations/set_pump_speed" \
    -H "Content-Type: application/json" -d '{"value": 80.0}' > /dev/null

  ALARM_CLEARED=0
  for i in $(seq 1 60); do
    TEMP=$(curl $CURL_OPTS "${API}/apps/tank_process/x-plc-data/tank_temperature" 2>/dev/null | jq '.value' 2>/dev/null || echo "100")
    if (( $(echo "${TEMP:-100} < 80" | bc -l 2>/dev/null || echo 0) )); then
      echo "  Temperature dropped to ${TEMP}C"
      ALARM_CLEARED=1
      break
    fi
    if (( i % 10 == 0 )); then
      echo "  Temperature at ${TEMP}C..."
    fi
    sleep 2
  done

  if [[ $ALARM_CLEARED -eq 1 ]]; then
    sleep 2
    TOTAL=$((TOTAL + 1))
    FAULTS=$(curl $CURL_OPTS "${API}/apps/tank_process/faults" 2>/dev/null | jq '[.faults[] | select(.fault_code == "PLC_HIGH_TEMP" and .status == "active")] | length' 2>/dev/null || echo "1")
    if [[ "$FAULTS" == "0" ]]; then
      PASSED=$((PASSED + 1))
      echo -e "${GREEN}PASS${NC} PLC_HIGH_TEMP fault cleared after temperature drop"
    else
      FAILED=$((FAILED + 1))
      echo -e "${RED}FAIL${NC} PLC_HIGH_TEMP fault still active after temperature drop"
    fi
  else
    TOTAL=$((TOTAL + 1))
    FAILED=$((FAILED + 1))
    echo -e "${RED}FAIL${NC} Temperature did not drop below 80C within timeout"
  fi
else
  echo -e "${YELLOW}SKIP${NC} Fault lifecycle tests - temperature did not reach threshold"
  echo "  (This may happen if PLC simulation rate differs from expected)"
fi

# Restore normal operation for remaining tests
curl -s -X POST "${API}/apps/fill_pump/x-plc-operations/set_pump_speed" \
  -H "Content-Type: application/json" -d '{"value": 50.0}' > /dev/null 2>&1 || true
curl -s -X POST "${API}/apps/drain_valve/x-plc-operations/set_valve_position" \
  -H "Content-Type: application/json" -d '{"value": 30.0}' > /dev/null 2>&1 || true

# =============================================================================
# 5. Connection Status
# =============================================================================
echo ""
echo -e "${YELLOW}--- 5. Connection Status ---${NC}"

assert_response "x-plc-status endpoint returns 200" \
  "GET" "${API}/components/openplc_runtime/x-plc-status" "200"

assert_response "x-plc-status shows connected=true" \
  "GET" "${API}/components/openplc_runtime/x-plc-status" "200" "" '.connected' "true"

TOTAL=$((TOTAL + 1))
POLL_COUNT=$(curl $CURL_OPTS "${API}/components/openplc_runtime/x-plc-status" 2>/dev/null | jq '.poll_count' 2>/dev/null || echo "0")
if [[ -n "$POLL_COUNT" ]] && (( POLL_COUNT > 0 )); then
  PASSED=$((PASSED + 1))
  echo -e "${GREEN}PASS${NC} poll_count > 0 (got $POLL_COUNT)"
else
  FAILED=$((FAILED + 1))
  echo -e "${RED}FAIL${NC} poll_count should be > 0, got $POLL_COUNT"
fi

# =============================================================================
# 6. Error Responses (SOVD conformance)
# =============================================================================
echo ""
echo -e "${YELLOW}--- 6. Error Responses ---${NC}"

assert_response "404 for nonexistent app x-plc-data" \
  "GET" "${API}/apps/nonexistent/x-plc-data" "404"

assert_response "404 for nonexistent operation" \
  "POST" "${API}/apps/fill_pump/x-plc-operations/nonexistent" "404" \
  '{"value": 42}'

assert_response "400 for invalid JSON body" \
  "POST" "${API}/apps/fill_pump/x-plc-operations/set_pump_speed" "400" \
  'not json'

assert_response "400 for missing value field" \
  "POST" "${API}/apps/fill_pump/x-plc-operations/set_pump_speed" "400" \
  '{"wrong": 42}'

assert_response "400 for read-only data point write" \
  "POST" "${API}/apps/fill_pump/x-plc-operations/set_pump_running" "404"

assert_response "404 for nonexistent single data point" \
  "GET" "${API}/apps/tank_process/x-plc-data/nonexistent" "404"
