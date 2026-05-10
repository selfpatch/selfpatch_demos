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

section "Fault round-trip via inject-collision"
EXEC_BODY='{"execution_type": "now"}'
if curl -fsS -X POST -H "Content-Type: application/json" -d "$EXEC_BODY" \
    "$API_BASE/components/manymove-planning/scripts/inject-collision/executions" >/dev/null; then
    pass "inject-collision script accepted by gateway"
else
    fail "gateway rejected inject-collision script execution"
fi

# Give the BT a few ticks to observe the blackboard flag, run a retry cycle
# and emit either the collision-detected fault (if MoveManipulator::onStart
# is the next BT node) or the retries-exhausted fault.
sleep 6

# Either kPlannerCollisionDetected or kPlannerRetriesExhausted is acceptable:
# both prove the BT round-trip works, just from different code paths in
# MoveManipulatorAction.
if poll_until "/faults" \
    '.items[] | select(.fault_code == "MANYMOVE_PLANNER_COLLISION_DETECTED" or .fault_code == "MANYMOVE_PLANNER_RETRIES_EXHAUSTED")' 30; then
    pass "manymove planner fault visible in fault list"
else
    fail "no MANYMOVE_PLANNER_* fault appeared after inject-collision"
fi

section "Restore"
if curl -fsS -X POST -H "Content-Type: application/json" -d "$EXEC_BODY" \
    "$API_BASE/components/manymove-planning/scripts/restore-normal/executions" >/dev/null; then
    pass "restore-normal script accepted"
else
    fail "restore-normal script rejected"
fi

# Final summary
echo ""
if [ "$FAIL_COUNT" -gt 0 ]; then
    echo -e "${RED}Failed: $FAIL_COUNT${NC} (passed: $PASS_COUNT)"
    echo -e "Failed tests:$FAILED_TESTS"
    exit 1
fi
echo -e "${GREEN}All $PASS_COUNT smoke tests passed${NC}"
