#!/bin/bash
# Restore Normal Operation - Remove all injected faults
set -eu

CONTAINER="${CONTAINER_NAME:-$(docker ps --format '{{.Names}}' | grep -E '^moveit_medkit_demo(_nvidia)?(_local)?$' | head -n1)}"
if [ -z "${CONTAINER}" ]; then
    echo "❌ Demo container not running. Start it first: ./run-demo.sh"
    exit 1
fi

exec docker exec -it "${CONTAINER}" restore-normal.sh
