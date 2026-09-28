#!/bin/bash
# Smoke tests for turtlebot3_integration demo
# Runs from the host against the containerized gateway on localhost:8080
#
# Tests: health, entity discovery (areas/components/apps/functions),
#   discovery relationships, Linux introspection, data access, operations,
#   configurations, scripts (list + execution), bulk data, faults, logs,
#   trigger CRUD lifecycle, check-entities.sh and check-faults.sh with one
#   injected navigation failure
#
# Usage: ./tests/smoke_test_turtlebot3.sh [GATEWAY_URL]
# Default GATEWAY_URL: http://localhost:8080

GATEWAY_URL="${1:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=tests/smoke_lib.sh
source "${SCRIPT_DIR}/smoke_lib.sh"

trap print_summary EXIT

# --- Wait for gateway startup ---

# Turtlebot3 needs Gazebo + Nav2 - allow extra startup time
wait_for_gateway 120

# Wait for runtime node linking
wait_for_runtime_linking "/apps/medkit-gateway/data" 90

# --- Tests ---

section "Health"

if api_get "/health"; then
    pass "GET /health returns 200"
else
    fail "GET /health returns 200" "unexpected status code"
fi

test_entity_discovery "areas" robot navigation diagnostics bridge
test_entity_discovery "components" turtlebot3-base lidar-sensor nav2-stack gateway fault-manager diagnostic-bridge-unit
test_entity_discovery "apps" turtlebot3-node robot-state-publisher gazebo amcl bt-navigator controller-server planner-server velocity-smoother medkit-gateway medkit-fault-manager diagnostic-bridge anomaly-detector
test_entity_discovery "functions" autonomous-navigation robot-control fault-management

section "Discovery Relationships"

assert_non_empty_items "/areas/robot/components"

section "Linux Introspection"

assert_procfs_introspection "medkit-gateway"

section "Data Access"

assert_non_empty_items "/apps/medkit-gateway/data"

section "Operations"

# fault_manager services may take extra time to be discovered in Gazebo-heavy demos
echo "  Waiting for fault-manager operations to appear (max 30s)..."
if poll_until "/apps/medkit-fault-manager/operations" '.items | length > 0' 30; then
    pass "GET /apps/medkit-fault-manager/operations returns non-empty items"
else
    fail "GET /apps/medkit-fault-manager/operations returns non-empty items" "items still empty after 30s"
fi

section "Configurations"

assert_non_empty_items "/apps/medkit-gateway/configurations"

section "Scripts"

assert_scripts_list "nav2-stack" "nav-health-check"
assert_script_execution "nav2-stack" "nav-health-check" 30

section "Bulk Data"

if api_get "/apps/diagnostic-bridge/bulk-data"; then
    pass "GET /apps/diagnostic-bridge/bulk-data returns 200"
else
    fail "GET /apps/diagnostic-bridge/bulk-data returns 200" "unexpected status code"
fi

section "Faults"

if api_get "/faults"; then
    pass "GET /faults returns 200"
else
    fail "GET /faults returns 200" "unexpected status code"
fi

section "Logs"

assert_non_empty_items "/apps/medkit-gateway/logs"

section "Check-Entities and Check-Faults Scripts"

TB3_DIR="${SCRIPT_DIR}/../demos/turtlebot3_integration"

# Echoes a Nav2 node's lifecycle state label, or "unavailable".
# Usage: lifecycle_label APP
lifecycle_label() {
    curl -s -m 20 -X POST "${API_BASE}/apps/$1/operations/get_state/executions" \
        -H 'Content-Type: application/json' -d '{"parameters":{}}' 2>/dev/null \
        | jq -r '.parameters.current_state.label // "unavailable"' 2>/dev/null || echo unavailable
}

# The inject needs a goal that Nav2 accepts and then aborts. An inactive
# navigator rejects the goal and no fault follows. Nav2 activates well after
# the gateway answers, so wait for the navigator and the planner.
echo "  Waiting for bt-navigator and planner-server to be active (max 180s)..."
nav2_state=""
elapsed=0
while [ "$elapsed" -lt 180 ]; do
    nav2_state="$(lifecycle_label bt-navigator)/$(lifecycle_label planner-server)"
    [ "$nav2_state" = "active/active" ] && break
    sleep 5
    elapsed=$((elapsed + 5))
done
if [ "$nav2_state" = "active/active" ]; then
    pass "bt-navigator and planner-server are active before the inject"
else
    fail "bt-navigator and planner-server are active before the inject" "states: ${nav2_state}"
fi

# Inject a real fault via the Scripts API so check-entities.sh (section 6)
# and check-faults.sh exercise the fault-carrying fields, not just the
# empty case.
echo "  Injecting navigation failure via Scripts API..."
INJECT_RESPONSE=$(curl -s -m 30 -X POST "${API_BASE}/components/nav2-stack/scripts/inject-nav-failure/executions" \
    -H "Content-Type: application/json" -d '{"execution_type": "now"}') || true
INJECT_EXEC_ID=$(echo "$INJECT_RESPONSE" | jq -r '.id // empty')
if [ -n "$INJECT_EXEC_ID" ]; then
    elapsed=0
    while [ $elapsed -lt 30 ]; do
        st=$(curl -s "${API_BASE}/components/nav2-stack/scripts/inject-nav-failure/executions/${INJECT_EXEC_ID}" | jq -r '.status')
        if [ "$st" = "completed" ] || [ "$st" = "failed" ]; then
            break
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done
fi

echo "  Waiting for NAVIGATION_GOAL_ABORTED fault to appear (max 15s)..."
if poll_until "/faults" '.items[] | select(.fault_code == "NAVIGATION_GOAL_ABORTED")' 15; then
    pass "NAVIGATION_GOAL_ABORTED fault appeared in /faults"
else
    fail "NAVIGATION_GOAL_ABORTED fault appeared in /faults" "fault not found after 15s"
fi

# Prints the JSON values printed in section N of a script output as one array:
# [] when the section is missing, nothing when it is not JSON. Text after the
# section's last closing bracket, such as a script's closing lines, is dropped.
# Usage: script_section OUTPUT N
script_section() {
    awk -v hdr="^=== $2\\\\. " '
        $0 ~ hdr { on = 1; next }
        on && /^=== [0-9]+\. / { exit }
        on { lines[++n] = $0; if ($0 ~ /^[]}]/) last = n }
        END { for (i = 1; i <= last; i++) print lines[i] }' <<< "$1" \
        | sed -n '/^[[{]/,$p' | jq -s '.' 2>/dev/null || true
}

# Prints the fault list as the scripts show it, normalised by FILTER.
# Usage: api_faults FILTER
api_faults() {
    curl -s -m 20 "${API_BASE}/faults" | jq -c "[.items[] | $1] | sort_by(.code)" 2>/dev/null || true
}

# Passes when PRINTED is non-empty and equals EXPECTED or ALTERNATIVE.
# Usage: assert_printed_matches DESCRIPTION PRINTED EXPECTED [ALTERNATIVE]
assert_printed_matches() {
    local description="$1" printed="$2" expected="$3" alternative="${4:-}"
    if [ -n "$printed" ] && { [ "$printed" = "$expected" ] || [ "$printed" = "$alternative" ]; }; then
        pass "$description"
    else
        fail "$description" "printed ${printed:-nothing}; API ${expected:-unreadable}"
    fi
}

# The fault list can change while a script runs, so it is read before and
# after; the printed faults must equal one of the two reads.
ENTITY_FAULT_FIELDS='{code: .fault_code, severity: .severity_label, sources: .reporting_sources}'
FAULT_FIELDS='{code: .fault_code, severity: .severity_label, status: .status,
    sources: .reporting_sources, occurrences: .occurrence_count}'

ENTITY_FAULTS_BEFORE=$(api_faults "$ENTITY_FAULT_FIELDS")
CHECK_ENTITIES_PLAIN=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash ./check-entities.sh 2>&1 \
    | sed 's/\x1b\[[0-9;]*m//g') || true
ENTITY_FAULTS_AFTER=$(api_faults "$ENTITY_FAULT_FIELDS")

if grep -q ': null' <<< "$CHECK_ENTITIES_PLAIN"; then
    fail "check-entities.sh prints no null fields" "$(grep -B1 ': null' <<< "$CHECK_ENTITIES_PLAIN" | head -10)"
else
    pass "check-entities.sh prints no null fields"
fi

if grep -q "NAVIGATION_GOAL_ABORTED" <<< "$CHECK_ENTITIES_PLAIN"; then
    pass "check-entities.sh faults section shows the active fault code"
else
    fail "check-entities.sh faults section shows the active fault code" "NAVIGATION_GOAL_ABORTED not in output"
fi

assert_printed_matches "check-entities.sh lists every component id" \
    "$(script_section "$CHECK_ENTITIES_PLAIN" 2 | jq -c '[.[].id] | sort' 2>/dev/null)" \
    "$(curl -s -m 20 "${API_BASE}/components" | jq -c '[.items[].id] | sort' 2>/dev/null)"

assert_printed_matches "check-entities.sh shows each app with its component" \
    "$(script_section "$CHECK_ENTITIES_PLAIN" 3 | jq -c 'map({id, component}) | sort_by(.id)' 2>/dev/null)" \
    "$(curl -s -m 20 "${API_BASE}/apps" \
        | jq -c '[.items[] | {id, component: .["x-medkit"].component_id}] | sort_by(.id)' 2>/dev/null)"

assert_printed_matches "check-entities.sh shows every active fault with its severity and sources" \
    "$(script_section "$CHECK_ENTITIES_PLAIN" 6 | jq -c 'map({code, severity, sources}) | sort_by(.code)' 2>/dev/null)" \
    "$ENTITY_FAULTS_BEFORE" "$ENTITY_FAULTS_AFTER"

FAULTS_BEFORE=$(api_faults "$FAULT_FIELDS")
CHECK_FAULTS_PLAIN=$(cd "$TB3_DIR" && GATEWAY_URL="$GATEWAY_URL" bash ./check-faults.sh 2>&1 \
    | sed 's/\x1b\[[0-9;]*m//g') || true
FAULTS_AFTER=$(api_faults "$FAULT_FIELDS")

if grep -q ': null' <<< "$CHECK_FAULTS_PLAIN"; then
    fail "check-faults.sh prints no null fields" "$(grep -B1 ': null' <<< "$CHECK_FAULTS_PLAIN" | head -10)"
else
    pass "check-faults.sh prints no null fields"
fi

if grep -q "NAVIGATION_GOAL_ABORTED" <<< "$CHECK_FAULTS_PLAIN"; then
    pass "check-faults.sh shows the active fault code"
else
    fail "check-faults.sh shows the active fault code" "NAVIGATION_GOAL_ABORTED not in output"
fi

# check-faults.sh prints the faults between its "Active Faults:" and
# "Fault Summary:" lines.
assert_printed_matches "check-faults.sh shows every active fault with its severity, status, sources and count" \
    "$(awk '/Active Faults:$/ { on = 1; next } /Fault Summary:$/ { exit } on' <<< "$CHECK_FAULTS_PLAIN" \
        | sed -n '/^[[{]/,$p' | jq -cs 'map({code, severity, status, sources, occurrences}) | sort_by(.code)' 2>/dev/null)" \
    "$FAULTS_BEFORE" "$FAULTS_AFTER"

# Cleanup: clear all faults so smoke_test_navigation.sh (run next on this
# stack) does not inherit a latched fault confirmation.
echo "  Cleaning up: clearing faults..."
curl -s -X DELETE "${API_BASE}/faults" > /dev/null || true

section "Triggers"

assert_triggers_crud "apps" "diagnostic-bridge" "/api/v1/apps/diagnostic-bridge/faults"

# --- Summary ---

# print_summary runs via EXIT trap; exit code reflects test results
[ "$FAIL_COUNT" -eq 0 ]
