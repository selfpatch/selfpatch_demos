#!/usr/bin/env bash
# Inject faults into the OpenPLC Tank Demo
# Writes values directly to PLC via the x-plc-operations endpoint
set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

FAULT_TYPE="${1:-high_temp}"

usage() {
  echo "Usage: $0 [FAULT_TYPE]"
  echo ""
  echo "Fault types:"
  echo "  high_temp     - Set temperature to 95C (triggers PLC_HIGH_TEMP alarm)"
  echo "  low_level     - Stop pump + open drain (triggers PLC_LOW_LEVEL alarm)"
  echo "  overpressure  - High pump speed to increase pressure (triggers PLC_OVERPRESSURE)"
  echo "  all           - Inject all faults simultaneously"
  echo ""
  echo "Default: high_temp"
}

inject_high_temp() {
  echo "Injecting HIGH_TEMP fault..."
  echo "  Setting pump speed to 0% (stop cooling)"
  curl -s -X POST "${API_BASE}/apps/fill_pump/x-plc-operations/set_pump_speed" \
    -H "Content-Type: application/json" \
    -d '{"value": 0.0}' | jq .

  echo "  Temperature will rise above 80C threshold..."
  echo "  Watch: curl ${API_BASE}/apps/tank_process/x-plc-data/tank_temperature"
  echo "  Fault: curl ${API_BASE}/apps/tank_process/faults"
}

inject_low_level() {
  echo "Injecting LOW_LEVEL fault..."
  echo "  Stopping pump and opening drain valve to 100%"
  curl -s -X POST "${API_BASE}/apps/fill_pump/x-plc-operations/set_pump_speed" \
    -H "Content-Type: application/json" \
    -d '{"value": 0.0}' | jq .

  curl -s -X POST "${API_BASE}/apps/drain_valve/x-plc-operations/set_valve_position" \
    -H "Content-Type: application/json" \
    -d '{"value": 100.0}' | jq .

  echo "  Tank level will drop below 100mm threshold..."
  echo "  Watch: curl ${API_BASE}/apps/tank_process/x-plc-data/tank_level"
}

inject_overpressure() {
  echo "Injecting OVERPRESSURE fault..."
  echo "  Setting pump to 100% and closing drain valve"
  curl -s -X POST "${API_BASE}/apps/fill_pump/x-plc-operations/set_pump_speed" \
    -H "Content-Type: application/json" \
    -d '{"value": 100.0}' | jq .

  curl -s -X POST "${API_BASE}/apps/drain_valve/x-plc-operations/set_valve_position" \
    -H "Content-Type: application/json" \
    -d '{"value": 0.0}' | jq .

  echo "  Pressure will rise above 5 bar threshold..."
  echo "  Watch: curl ${API_BASE}/apps/tank_process/x-plc-data/tank_pressure"
}

case "$FAULT_TYPE" in
  high_temp)     inject_high_temp ;;
  low_level)     inject_low_level ;;
  overpressure)  inject_overpressure ;;
  all)
    inject_high_temp
    echo ""
    inject_low_level
    echo ""
    inject_overpressure
    ;;
  -h|--help)     usage ;;
  *)             echo "Unknown fault type: $FAULT_TYPE"; usage; exit 1 ;;
esac

echo ""
echo "Check faults: curl ${API_BASE}/faults | jq ."
