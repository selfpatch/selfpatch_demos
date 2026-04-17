#!/bin/bash
# Trigger the LiDAR HIGH_NOISE fault on the running single-robot stack.
#
# Calls the medkit gateway scripts API to execute the inject-noise script,
# which sets lidar-sim noise_stddev=0.5 and camera-sim noise_level=0.3.
# That makes lidar_sim publish noisy ranges + a HIGH_NOISE diagnostic,
# diagnostic_bridge picks it up and reports a fault to fault_manager,
# fault_manager confirms it (confirmation_threshold=0), the rosbag ring
# buffer is flushed, and a few seconds later the bridge container picks
# up the fault_confirmed SSE event and ingests the bag into mosaicod.

set -euo pipefail

GATEWAY_URL="${GATEWAY_URL:-http://localhost:18080}"
API_BASE="${GATEWAY_URL}/api/v1"

# Dependency check
for cmd in curl jq; do
    if ! command -v "$cmd" >/dev/null 2>&1; then
        echo "Required tool '$cmd' is not installed."
        exit 1
    fi
done

# Make sure the gateway is up
if ! curl -sf "${API_BASE}/health" > /dev/null; then
    echo "Gateway not reachable at ${GATEWAY_URL}."
    echo "Did you run 'docker compose up -d' from the demo directory?"
    exit 1
fi

echo ">> Triggering inject-noise on compute-unit"

# Start the script via the SOVD scripts API. medkit requires execution_type
# in the POST body.
START_RESP=$(curl -sf -X POST "${API_BASE}/components/compute-unit/scripts/inject-noise/executions" \
    -H "Content-Type: application/json" -d '{"execution_type": "now"}')

EXEC_ID=$(echo "$START_RESP" | jq -r '.id // .execution_id // empty')
if [ -z "$EXEC_ID" ]; then
    echo "Could not parse execution id from response:"
    echo "$START_RESP"
    exit 1
fi
echo "   execution id: $EXEC_ID"

# Poll until the script finishes (max 30s)
completed=0
STATUS="unknown"
for _ in $(seq 1 30); do
    STATUS=$(curl -sf "${API_BASE}/components/compute-unit/scripts/inject-noise/executions/${EXEC_ID}" \
        | jq -r '.status // "unknown"')
    case "$STATUS" in
        completed|succeeded|success)
            echo ">> inject-noise completed"
            completed=1
            break
            ;;
        failed|error)
            echo ">> inject-noise FAILED (status=$STATUS)" >&2
            exit 1
            ;;
        *)
            sleep 1
            ;;
    esac
done

if [ "$completed" -ne 1 ]; then
    echo ">> inject-noise TIMED OUT after 30s (last status=$STATUS)" >&2
    echo "   The gateway did not report completed/succeeded/success before the deadline." >&2
    exit 1
fi

echo ""
echo "Fault injected. The bridge waits POST_FAULT_WAIT_SEC (default 12s)"
echo "after fault_confirmed before downloading the bag, so expect the"
echo "fault_confirmed event and ingest ~15-20s from now. Watch the bridge logs:"
echo ""
echo "   docker compose logs -f bridge"
echo ""
echo "Then query mosaicod from a notebook against:"
echo ""
echo "   MosaicoClient.connect(host='localhost', port=16726)"
