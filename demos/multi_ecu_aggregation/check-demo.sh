#!/bin/bash
# Multi-ECU Aggregation Demo - Readiness Check
# Waits for the gateway to become healthy, verifies peer connections, and prints entity counts

set -eu

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"
TIMEOUT=60

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

echo_success() { echo -e "${GREEN}ok${NC}  $1"; }
echo_fail()    { echo -e "${RED}FAIL${NC}  $1"; }
echo_warn()    { echo -e "${YELLOW}WARN${NC}  $1"; }
echo_info()    { echo -e "${BLUE}--${NC}  $1"; }

# Check dependencies
for cmd in curl jq; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo_fail "Required tool '$cmd' is not installed."
        exit 1
    fi
done

echo ""
echo -e "${BOLD}Multi-ECU Aggregation Demo - Readiness Check${NC}"
echo "=============================================="
echo ""

# --- 1. Poll health endpoint until it responds ---
echo -n "Waiting for gateway at ${GATEWAY_URL} "
elapsed=0
while [ "$elapsed" -lt "$TIMEOUT" ]; do
    if curl -sf "${API_BASE}/health" >/dev/null 2>&1; then
        break
    fi
    echo -n "."
    sleep 2
    elapsed=$((elapsed + 2))
done
echo ""

if [ "$elapsed" -ge "$TIMEOUT" ]; then
    echo_fail "Gateway did not respond within ${TIMEOUT}s"
    echo "   Start the demo first: ./run-demo.sh"
    exit 1
fi

HEALTH_JSON=$(curl -sf "${API_BASE}/health")
status=$(echo "$HEALTH_JSON" | jq -r '.status')
uptime=$(echo "$HEALTH_JSON" | jq -r '.uptime_seconds // "n/a"')
echo_success "Gateway is healthy (status=${status}, uptime=${uptime}s)"
echo ""

# --- 2. Check aggregation peers ---
echo -e "${BOLD}Peer Connections${NC}"
echo "----------------"

PEER_COUNT=$(echo "$HEALTH_JSON" | jq '[.peers // [] | .[] ] | length')

if [ "$PEER_COUNT" -ge 2 ]; then
    echo_success "${PEER_COUNT} peers connected"
else
    echo_warn "Expected 2 peers, found ${PEER_COUNT}"
fi

# Print individual peer status
echo "$HEALTH_JSON" | jq -r '
    .peers // [] | .[] |
    "  \(.name // .url) - \(.status // "unknown")"
' 2>/dev/null || true

echo ""

# --- 3. Entity counts ---
echo -e "${BOLD}Entity Counts${NC}"
echo "-------------"

for entity in areas components apps functions; do
    response=$(curl -sf "${API_BASE}/${entity}" 2>/dev/null) || response=""
    if [ -n "$response" ]; then
        count=$(echo "$response" | jq '.items | length')
        echo_info "${entity}: ${count}"
    else
        echo_warn "${entity}: endpoint unavailable"
    fi
done

echo ""

# --- 4. Summary ---
echo -e "${BOLD}Summary${NC}"
echo "-------"

if [ "$PEER_COUNT" -ge 2 ]; then
    echo_success "Demo is ready - all peers connected"
    echo ""
    echo "  Web UI:   http://localhost:3000"
    echo "  REST API: ${API_BASE}/"
    echo ""
    echo "  Try:"
    echo "    curl ${API_BASE}/components | jq '.items[].id'"
    echo "    curl ${API_BASE}/functions | jq '.items[].id'"
    echo "    ./inject-cascade-failure.sh"
    exit 0
else
    echo_fail "Demo is not fully ready - waiting for peers"
    echo ""
    echo "  Peers may still be booting. Retry in a few seconds:"
    echo "    ./check-demo.sh"
    echo ""
    echo "  To inspect peer config:"
    echo "    curl ${API_BASE}/health | jq '.peers'"
    exit 1
fi
