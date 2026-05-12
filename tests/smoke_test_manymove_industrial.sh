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
# service. This exercises the medkit gateway + manager pipeline without
# depending on BT trajectory state.
if curl -fsS -X POST -H "Content-Type: application/json" -d "$EXEC_BODY" \
    "$API_BASE/components/manymove-planning/scripts/arm-self-test/executions" >/dev/null; then
    pass "arm-self-test script accepted by gateway"
else
    fail "gateway rejected arm-self-test script execution"
fi

# arm-self-test sleeps 1s between FAILED and PASSED, so the fault should
# briefly appear in /faults as CONFIRMED before HEALED. Poll the historical
# list (statuses=all) to catch it regardless of current state.
if poll_until "/faults?statuses=CONFIRMED,HEALED" \
    '.items[] | select(.fault_code == "MANYMOVE_SELFTEST")' 30; then
    pass "MANYMOVE_SELFTEST fault round-tripped through medkit"
else
    fail "MANYMOVE_SELFTEST fault did not appear in fault list"
fi

# Final summary
echo ""
if [ "$FAIL_COUNT" -gt 0 ]; then
    echo -e "${RED}Failed: $FAIL_COUNT${NC} (passed: $PASS_COUNT)"
    echo -e "Failed tests:$FAILED_TESTS"
    exit 1
fi
echo -e "${GREEN}All $PASS_COUNT smoke tests passed${NC}"
