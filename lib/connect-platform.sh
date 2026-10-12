#!/bin/bash
# Connect a running demo to the selfpatch platform through the selfpatch agent.
# Source-free: each demo's connect-platform.sh calls this with its gateway
# container name.
#
#   lib/connect-platform.sh <gateway-container>             pair and start the agent
#   lib/connect-platform.sh <gateway-container> --disconnect stop and remove the agent
#
# The agent runs as its own container on the demo's Docker network, opens an
# outbound connection to the platform and lets it read the demo's gateway. It
# keeps its credential in a volume, so it survives restarts and reconnects on
# its own. Each demo instance gets its own agent and shows up in the platform
# as its own device.
#
# Environment variables:
#   PLATFORM_URL          - platform to pair with (default: https://platform.selfpatch.ai)
#   PAIRING_CODE          - pairing code from the platform (asked for when unset)
#   SELFPATCH_AGENT_IMAGE - agent image (default: ghcr.io/selfpatch/agent:0.1)
#   PLATFORM_INSECURE_HTTP=1 - allow a plain http platform, e.g. a local dev
#                           platform at http://host.docker.internal:<port>

set -eu

usage() {
    echo "Usage: $0 <gateway-container> [--disconnect]"
}

if [[ $# -lt 1 ]]; then
    usage
    exit 1
fi

GATEWAY_CONTAINER="$1"
ACTION="${2:-connect}"
PLATFORM_URL="${PLATFORM_URL:-https://platform.selfpatch.ai}"
AGENT_IMAGE="${SELFPATCH_AGENT_IMAGE:-ghcr.io/selfpatch/agent:0.1}"
AGENT_NAME="selfpatch-agent-${GATEWAY_CONTAINER}"

if [[ "$ACTION" == "--disconnect" ]]; then
    docker rm -f "$AGENT_NAME" >/dev/null 2>&1 || true
    docker volume rm "$AGENT_NAME" >/dev/null 2>&1 || true
    echo "Agent ${AGENT_NAME} removed. Remove the device in the platform too."
    exit 0
fi

if [[ "$(docker inspect -f '{{.State.Running}}' "$GATEWAY_CONTAINER" 2>/dev/null)" != "true" ]]; then
    echo "Container ${GATEWAY_CONTAINER} is not running. Start the demo first: ./run-demo.sh"
    exit 1
fi

NETWORK="$(docker inspect -f '{{range $name, $_ := .NetworkSettings.Networks}}{{$name}} {{end}}' "$GATEWAY_CONTAINER" | awk '{print $1}')"
if [[ -z "$NETWORK" || "$NETWORK" == "host" ]]; then
    MEDKIT_URL="http://127.0.0.1:8080"
    NETWORK="host"
else
    MEDKIT_URL="http://${GATEWAY_CONTAINER}:8080"
fi

if ! docker image inspect "$AGENT_IMAGE" >/dev/null 2>&1 && ! docker pull -q "$AGENT_IMAGE" >/dev/null; then
    echo "Cannot pull ${AGENT_IMAGE}. Set SELFPATCH_AGENT_IMAGE to an agent image you can reach."
    exit 1
fi

if [[ -z "${PAIRING_CODE:-}" ]]; then
    echo "In the platform: Devices > Add device > 'Through the selfpatch agent', then copy the pairing code."
    read -r -p "Pairing code: " PAIRING_CODE
fi

PAIR_ARGS=(pair --platform-url "$PLATFORM_URL" --medkit-url "$MEDKIT_URL")
RUN_ARGS=(run)
if [[ "${PLATFORM_INSECURE_HTTP:-0}" == "1" ]]; then
    PAIR_ARGS+=(--insecure-platform-http)
    RUN_ARGS+=(--insecure-platform-http)
fi

docker volume create "$AGENT_NAME" >/dev/null
echo "$PAIRING_CODE" | docker run --rm -i --network "$NETWORK" \
    -e SELFPATCH_AGENT_STATE=/data -v "${AGENT_NAME}:/data" \
    "$AGENT_IMAGE" "${PAIR_ARGS[@]}"

docker rm -f "$AGENT_NAME" >/dev/null 2>&1 || true
docker run -d --name "$AGENT_NAME" --restart unless-stopped --network "$NETWORK" \
    -e SELFPATCH_AGENT_STATE=/data -v "${AGENT_NAME}:/data" \
    "$AGENT_IMAGE" "${RUN_ARGS[@]}" >/dev/null

echo ""
echo "Agent ${AGENT_NAME} is running. The demo appears in the platform within a few seconds."
echo "Logs:       docker logs -f ${AGENT_NAME}"
echo "Disconnect: GATEWAY_CONTAINER=${GATEWAY_CONTAINER} ./connect-platform.sh --disconnect (in the demo directory)"
