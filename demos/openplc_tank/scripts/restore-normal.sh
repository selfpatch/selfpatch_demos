#!/usr/bin/env bash
# Restore normal operation of the OpenPLC Tank Demo
# Sets pump and valve to safe operating values
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

echo "=== Restoring Normal Operation ==="

echo "Setting pump speed to 50%..."
curl -s -X POST "${API_BASE}/apps/fill_pump/x-plc-operations/set_pump_speed" \
  -H "Content-Type: application/json" \
  -d '{"value": 50.0}' > /dev/null

echo "Setting valve position to 30%..."
curl -s -X POST "${API_BASE}/apps/drain_valve/x-plc-operations/set_valve_position" \
  -H "Content-Type: application/json" \
  -d '{"value": 30.0}' > /dev/null

echo ""
echo "Normal operating parameters set."
echo "  Pump: 50% (filling)"
echo "  Valve: 30% (partial drain)"
echo ""
echo "Tank will stabilize to normal levels."
echo "Alarms will clear automatically when values return to safe range."
echo ""

# Clear any existing faults from fault manager
echo "Clearing existing faults..."
for code in PLC_HIGH_TEMP PLC_LOW_LEVEL PLC_OVERPRESSURE; do
  curl -s -X DELETE "${API_BASE}/apps/tank_process/faults/${code}" > /dev/null 2>&1 || true
done

echo "Done. Check status:"
echo "  curl ${API_BASE}/apps/tank_process/x-plc-data | jq ."
echo "  curl ${API_BASE}/faults | jq ."
