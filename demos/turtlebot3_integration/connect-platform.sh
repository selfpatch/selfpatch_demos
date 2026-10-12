#!/bin/bash
# Connect this demo to the selfpatch platform through the selfpatch agent.
# See lib/connect-platform.sh for the environment variables.
#
#   ./connect-platform.sh               pair and start the agent
#   ./connect-platform.sh --disconnect  stop and remove the agent

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# run-demo.sh --nvidia starts the gateway under a different container name.
if [[ -z "${GATEWAY_CONTAINER:-}" ]]; then
    GATEWAY_CONTAINER=turtlebot3_medkit_demo
    if [[ "$(docker inspect -f '{{.State.Running}}' turtlebot3_medkit_demo_nvidia 2>/dev/null)" == "true" ]]; then
        GATEWAY_CONTAINER=turtlebot3_medkit_demo_nvidia
    fi
fi
exec "${SCRIPT_DIR}/../../lib/connect-platform.sh" "$GATEWAY_CONTAINER" "$@"
