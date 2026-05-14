#!/bin/bash
# Smoke test for the manymove_industrial demo (CI profile).

set -euo pipefail

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

cd "$(dirname "$0")/.."

# shellcheck disable=SC1091
source ./tests/smoke_lib.sh

wait_for_gateway 90

section "Manifest discovery"
if api_get "/apps"; then
    if echo "$RESPONSE" | items_contain_id "bt-client-xarm7"; then
        pass "bt-client-xarm7 declared in apps"
    else
        fail "bt-client-xarm7 missing from apps list"
    fi

    if echo "$RESPONSE" | items_contain_id "fault-manager-app"; then
        pass "fault-manager-app declared in apps"
    else
        fail "fault-manager-app missing from apps list"
    fi
fi

section "Script bindings"
# Each script execution endpoint should resolve (proves manifest component
# id matches the container_scripts directory layout). We do not assert on
# BT-emitted faults here: BT-side fault emission requires an active move
# trajectory, and the CI fake-hardware launch does not auto-start moves.
# Real BT round-trip is covered by record_full.sh demo runs.
EXEC_BODY='{"execution_type": "now"}'
for script in inject-collision restore-normal; do
    if curl -fsS -X POST -H "Content-Type: application/json" -d "$EXEC_BODY" \
        "$API_BASE/components/manymove-planning/scripts/$script/executions" >/dev/null; then
        pass "$script script accepted by gateway"
    else
        fail "gateway rejected $script script execution"
    fi
done

section "Medkit REST -> FaultManager round-trip"
# arm-self-test issues a FAILED + PASSED pair directly via the FaultManager
# service. We only assert the script is accepted by the gateway; the
# fault visibility is intentionally not asserted because the test uses
# severity 0 (INFO) which does not pass the FaultManager debounce
# threshold, and even when it did the FAILED -> PASSED pair clears the
# fault from the active list within the smoke poll window.
# The full REST -> FaultManager round-trip is covered by the PLC bridge
# section below (real fault from a real source, severity 1 WARN).
if curl -fsS -X POST -H "Content-Type: application/json" -d "$EXEC_BODY" \
    "$API_BASE/components/manymove-planning/scripts/arm-self-test/executions" >/dev/null; then
    pass "arm-self-test script accepted by gateway"
else
    fail "gateway rejected arm-self-test script execution"
fi

section "PLC bridge: cross-source fault aggregation"
# Conveyor-line scripts trigger PLC AlarmConditionType events on the
# plc-sim OPC UA server; opcua_bridge subscribes and forwards them as
# MANYMOVE_PLC_* faults with source_id=/plc/sensor_io.

# Manifest must declare the conveyor-line component (the container_scripts
# directory + manifest component id must match for the gateway to expose
# the script endpoints).
if api_get "/components"; then
    if echo "$RESPONSE" | items_contain_id "openplc"; then
        pass "openplc component declared in components"
    else
        fail "openplc component missing from components list"
    fi
    if echo "$RESPONSE" | items_contain_id "opcua-bridge"; then
        pass "opcua-bridge component declared in components"
    else
        fail "opcua-bridge component missing from components list"
    fi
fi

# Wait for opcua_bridge to actually subscribe to AlarmConditionType
# events before injecting. Container manifest discovery (above) only
# proves the gateway routed the component, not that the asyncua
# subscription handshake on plc-sim completed. Without this wait the
# inject can fire 100-500ms before bridge is ready, the OPC UA raise
# event goes unheard and the test flakes.
#
# Skipped gracefully if the host docker CLI is not reachable (e.g.
# running the smoke script from inside a container without the socket
# mounted), since the belt-and-suspenders sleep below covers the race
# in practice.
# Probe once first; if docker logs of the bridge isn't reachable from
# this environment (script running from inside a container without the
# socket mounted, etc.) skip the check entirely - the sleep below
# covers the race for the common-case smoke runs.
PROBE=$(docker logs manymove_industrial-opcua-bridge-1 2>/dev/null || true)
if [ -n "$PROBE" ]; then
    BRIDGE_LOGS="$PROBE"
    for _ in $(seq 1 20); do
        if echo "$BRIDGE_LOGS" | grep -q "subscribed to AlarmConditionType events"; then
            pass "opcua_bridge subscribed to AlarmConditionType events"
            break
        fi
        sleep 1
        BRIDGE_LOGS=$(docker logs manymove_industrial-opcua-bridge-1 2>/dev/null || true)
    done
    if ! echo "$BRIDGE_LOGS" | grep -q "subscribed to AlarmConditionType events"; then
        fail "opcua_bridge never subscribed to AlarmConditionType events"
    fi
fi
# Belt-and-suspenders: small extra delay so subscription is fully active
# on the plc-sim side too (asyncua handshake completes ~50-200ms after
# the bridge logs the subscribe call).
sleep 1

# Photoeye flicker injection -> MANYMOVE_PLC_PHOTOEYE_FLICKER (WARN)
if curl -fsS -X POST -H "Content-Type: application/json" -d "$EXEC_BODY" \
    "$API_BASE/components/conveyor-line/scripts/inject-photoeye-flicker/executions" >/dev/null; then
    pass "inject-photoeye-flicker script accepted by gateway"
else
    fail "gateway rejected inject-photoeye-flicker script execution"
fi

if poll_until "/faults" \
    '.items[] | select(.fault_code == "MANYMOVE_PLC_PHOTOEYE_FLICKER")' 30; then
    pass "PLC photoeye flicker fault arrived via opcua_bridge"
else
    fail "PLC photoeye flicker fault did not arrive via opcua_bridge"
fi

# Clear PLC alarms -> PASSED events -> healed faults
if curl -fsS -X POST -H "Content-Type: application/json" -d "$EXEC_BODY" \
    "$API_BASE/components/conveyor-line/scripts/restore-line/executions" >/dev/null; then
    pass "restore-line script accepted by gateway"
else
    fail "gateway rejected restore-line script execution"
fi

if poll_until "/faults?statuses=HEALED,PREPASSED" \
    '.items[] | select(.fault_code == "MANYMOVE_PLC_PHOTOEYE_FLICKER")' 30; then
    pass "PLC photoeye flicker fault healed via opcua_bridge"
else
    fail "PLC photoeye flicker fault did not heal after restore-line"
fi

# Final summary
echo ""
if [ "$FAIL_COUNT" -gt 0 ]; then
    echo -e "${RED}Failed: $FAIL_COUNT${NC} (passed: $PASS_COUNT)"
    echo -e "Failed tests:$FAILED_TESTS"
    exit 1
fi
echo -e "${GREEN}All $PASS_COUNT smoke tests passed${NC}"
