#!/bin/bash
# Explore SOVD entity hierarchy from ros2_medkit gateway
# Demonstrates: Areas → Components → Apps → Functions

GATEWAY_URL="${GATEWAY_URL:-http://localhost:8080}"
API_BASE="${GATEWAY_URL}/api/v1"

# Colors for output
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo_step() {
    echo -e "\n${BLUE}=== $1 ===${NC}\n"
}

echo "╔════════════════════════════════════════════════════════════╗"
echo "║         SOVD Entity Hierarchy Explorer                     ║"
echo "║              TurtleBot3 + Nav2 Demo                        ║"
echo "╚════════════════════════════════════════════════════════════╝"

# Wait for gateway
echo ""
echo "Checking gateway health..."
if ! curl -sf "${API_BASE}/health" > /dev/null 2>&1; then
    echo "❌ Gateway not available at ${GATEWAY_URL}"
    echo "   Start with: ./run-demo.sh"
    exit 1
fi
echo "✓ Gateway is healthy"

echo_step "1. Areas (Namespace Groupings)"
curl -s "${API_BASE}/areas" | jq '.items[] | {id: .id, name: .name, description: .description}'

echo_step "2. Components (Hardware/Logical Units)"
curl -s "${API_BASE}/components" | jq '.items[] | {id: .id, name: .name, type: .type, area: .area}'

echo_step "3. Apps (ROS 2 Nodes)"
curl -s "${API_BASE}/apps" | jq '.items[] | {id: .id, name: .name, category: .category, component: .is_located_on}'

echo_step "4. Functions (High-level Capabilities)"
curl -s "${API_BASE}/functions" | jq '.items[] | {id: .id, name: .name, category: .category, hosted_by: .hosted_by}'

echo_step "5. Sample Data (LiDAR Scan)"
echo "Getting latest LiDAR scan from TurtleBot3..."
curl -s "${API_BASE}/apps/turtlebot3-node/data/scan" 2>/dev/null | jq '{
    angle_min: .angle_min,
    angle_max: .angle_max,
    range_min: .range_min,
    range_max: .range_max,
    sample_ranges: .ranges[:5]
}' || echo "   (LiDAR data not available - Gazebo may still be starting)"

echo_step "6. Faults"
curl -s "${API_BASE}/faults" | jq '.items[] | {code: .code, severity: .severity, reporter: .reporter_id}'

echo ""
echo -e "${GREEN}✓ Entity hierarchy exploration complete!${NC}"
echo ""
echo "Try more commands:"
echo "  curl ${API_BASE}/apps/amcl/configurations | jq    # AMCL parameters"
echo "  curl ${API_BASE}/apps/amcl/operations | jq        # Available services"
echo "  curl ${API_BASE}/components/nav2-stack/hosts | jq # Apps on Nav2 stack"
