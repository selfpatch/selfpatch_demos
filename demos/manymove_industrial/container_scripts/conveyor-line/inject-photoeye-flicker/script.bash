#!/bin/bash
# Raise PhotoeyeFlicker AlarmConditionType via the PLC admin endpoint.
# The opcua_bridge service is subscribed and forwards it as
# MANYMOVE_PLC_PHOTOEYE_FLICKER (WARN) to the medkit FaultManager.
set -e

PLC_ADMIN="${PLC_ADMIN_URL:-http://plc-sim:8500}"

curl -fsS -X POST -H "Content-Type: application/json" \
    -d '{"message":"Pick photoeye toggled 12 times in 2s, expected <=3"}' \
    "${PLC_ADMIN}/alarm/photoeye_flicker/raise" >/dev/null

echo "PhotoeyeFlicker alarm raised on ${PLC_ADMIN}"
