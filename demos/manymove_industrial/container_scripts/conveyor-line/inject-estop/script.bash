#!/bin/bash
# Raise EstopEngaged AlarmConditionType (CRITICAL severity) via the PLC
# admin endpoint. opcua_bridge forwards it to medkit as
# MANYMOVE_PLC_ESTOP_ENGAGED.
set -e

PLC_ADMIN="${PLC_ADMIN_URL:-http://plc-sim:8500}"

curl -fsS -X POST -H "Content-Type: application/json" \
    -d '{"message":"Line e-stop pressed at station 2"}' \
    "${PLC_ADMIN}/alarm/estop_engaged/raise" >/dev/null

echo "EstopEngaged alarm raised on ${PLC_ADMIN}"
