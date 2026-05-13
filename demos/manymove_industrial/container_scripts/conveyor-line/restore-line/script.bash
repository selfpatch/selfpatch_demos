#!/bin/bash
# Clear every PLC AlarmConditionType. opcua_bridge forwards a PASSED
# event per alarm, healing the corresponding medkit fault.
set -e

PLC_ADMIN="${PLC_ADMIN_URL:-http://plc-sim:8500}"

for alarm in photoeye_flicker conveyor_overspeed estop_engaged; do
    curl -fsS -X POST "${PLC_ADMIN}/alarm/${alarm}/clear" >/dev/null || true
    echo "cleared ${alarm}"
done
