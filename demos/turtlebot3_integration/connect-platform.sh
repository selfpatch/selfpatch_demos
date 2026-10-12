#!/bin/bash
# Connect this demo to the selfpatch platform through the selfpatch agent.
# See lib/connect-platform.sh for the environment variables.
#
#   ./connect-platform.sh               pair and start the agent
#   ./connect-platform.sh --disconnect  stop and remove the agent

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
exec "${SCRIPT_DIR}/../../lib/connect-platform.sh" "${GATEWAY_CONTAINER:-turtlebot3_medkit_demo}" "$@"
