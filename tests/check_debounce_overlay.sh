#!/bin/bash
# Check that the debounce override covers every service that can run the demo.
#
# A service missing from docker-compose.debounce.yml starts on the default
# profile while the caller believes it is debouncing. Nothing fails at that
# point: the demo comes up, the gateway answers, and every later assertion about
# thresholds quietly measures the wrong configuration. This is a static check on
# the merged compose file, so it needs no running container.
#
# Usage: ./tests/check_debounce_overlay.sh [demo_directory]

set -euo pipefail

DEMO_DIR="${1:-demos/turtlebot3_integration}"

for cmd in docker jq; do
    if ! command -v "$cmd" > /dev/null 2>&1; then
        echo "Error: required command '$cmd' not found in PATH" >&2
        exit 1
    fi
done

# Every service that launches demo.launch.py. Add a service here when you add one
# to docker-compose.yml.
DEMO_SERVICES=(turtlebot3-demo turtlebot3-demo-nvidia turtlebot3-demo-ci)

REQUIRED_TARGETS=(
    /root/demo_ws/src/turtlebot3_medkit_demo/config/medkit_params.yaml
    /root/demo_ws/src/turtlebot3_medkit_demo/config/entity_thresholds.yaml
)

MERGED=$(cd "$DEMO_DIR" && docker compose \
    --profile cpu --profile nvidia --profile ci \
    -f docker-compose.yml -f docker-compose.debounce.yml \
    config --format json)

failures=0

for service in "${DEMO_SERVICES[@]}"; do
    if ! jq -e --arg s "$service" '.services[$s]' > /dev/null 2>&1 <<< "$MERGED"; then
        echo "FAIL ${service}: not present in the merged compose file" >&2
        failures=$((failures + 1))
        continue
    fi
    for target in "${REQUIRED_TARGETS[@]}"; do
        if jq -e --arg s "$service" --arg t "$target" \
            '.services[$s].volumes // [] | map(.target) | index($t)' > /dev/null 2>&1 <<< "$MERGED"; then
            echo "PASS ${service} mounts ${target##*/}"
        else
            echo "FAIL ${service} does not mount ${target}" >&2
            failures=$((failures + 1))
        fi
    done
done

if [ "$failures" -ne 0 ]; then
    echo "${failures} missing debounce override mount(s)" >&2
    exit 1
fi

echo "All demo services carry the debounce override."
