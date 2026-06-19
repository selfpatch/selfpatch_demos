#!/usr/bin/env bash
# Copyright 2026 bburda
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Heap-on-Nav2: run the gateway under heaptrack while attached to the real Nav2
# stack (turtlebot3 demo), then decide leak-vs-cache from heaptrack's own
# heap-over-time series (a growing cache that is freed at exit does NOT show up
# as "leaked", so the time series is the honest signal, not the leaked total).
#
# Usage: heap_on_nav2.sh [DURATION_SEC] [WARMUP_SEC]
set -eo pipefail

DURATION="${1:-1500}"          # measurement window after warmup (default 25 min)
WARMUP="${2:-120}"             # let Nav2 + gateway settle before the window
BASE_IMAGE="${BASE_IMAGE:-bench_fp-turtlebot3-demo-ci:latest}"
IMAGE="medkit-heap-nav2:local"
NAME="heap_nav2"
SAMPLE_EVERY=30
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # benchmark/
OUT="${HERE}/results/heap_nav2_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

echo "== build heap-on-Nav2 image (rebuild gateway with symbols + heaptrack) =="
docker build -f "${HERE}/profiles/Dockerfile.benchmark-demo" \
    --build-arg "BASE=${BASE_IMAGE}" -t "$IMAGE" "${HERE}/profiles" >"$OUT/build.log" 2>&1 \
    || { echo "build failed; see $OUT/build.log"; tail -20 "$OUT/build.log"; exit 1; }

docker rm -f "$NAME" >/dev/null 2>&1 || true
echo "== launch Nav2 + gateway(heaptrack), domain 30 =="
docker run -d --name "$NAME" --shm-size=1g -e ROS_DOMAIN_ID=30 -e TURTLEBOT3_MODEL=burger \
    "$IMAGE" bash -c "mkdir -p /var/lib/ros2_medkit/rosbags /tmp/heap && \
        source /opt/ros/jazzy/setup.bash && source /root/demo_ws/install/setup.bash && \
        export TURTLEBOT3_MODEL=burger && \
        ros2 launch turtlebot3_medkit_demo demo.launch.py headless:=true" >/dev/null

cleanup() { docker rm -f "$NAME" >/dev/null 2>&1 || true; }
trap cleanup EXIT

echo "== wait for gateway HTTP ready =="
ready=0
for _ in $(seq 1 60); do
    if docker exec "$NAME" curl -fs -o /dev/null http://localhost:8080/api/v1/health 2>/dev/null; then
        ready=1; break
    fi
    sleep 5
done
[ "$ready" = 1 ] || { echo "gateway never became ready"; docker logs "$NAME" 2>&1 | tail -30; exit 1; }
echo "   ready; warmup ${WARMUP}s (let Nav2 + caches settle) ..."
sleep "$WARMUP"

# Resolve the gateway PID (for /proc USS sampling as a cross-check).
# The gateway runs as a child of heaptrack; its comm is exactly "gateway_node"
# (heaptrack's comm is "heaptrack"), so -x targets the real gateway, not the wrapper.
GPID="$(docker exec "$NAME" bash -c 'pgrep -x gateway_node | head -1' 2>/dev/null | tr -d '[:space:]')"
if [ -z "$GPID" ]; then
    echo "ERROR: could not resolve gateway_node PID inside container"
    docker logs "$NAME" 2>&1 | tail -20; exit 1
fi
echo "   gateway pid=${GPID}; measuring ${DURATION}s, USS sample every ${SAMPLE_EVERY}s"
echo "t_s,uss_kib" > "$OUT/uss.csv"
elapsed=0
while [ "$elapsed" -lt "$DURATION" ]; do
    uss="$(docker exec "$NAME" sh -c "awk '/^Private/{s+=\$2} END{print s}' /proc/${GPID}/smaps_rollup 2>/dev/null || awk '/^Private/{s+=\$2} END{print s}' /proc/${GPID}/smaps 2>/dev/null" 2>/dev/null | tr -d '[:space:]')"
    echo "${elapsed},${uss:-NA}" >> "$OUT/uss.csv"
    sleep "$SAMPLE_EVERY"; elapsed=$((elapsed + SAMPLE_EVERY))
done

echo "== stop gateway gracefully so heaptrack finalizes =="
# SIGINT ONLY the gateway child (comm=gateway_node), NOT the heaptrack wrapper
# (whose cmdline also contains "gateway_node") - so heaptrack observes the child
# exit and writes its trace instead of being killed mid-finalize.
docker exec "$NAME" bash -c 'pkill -INT -x gateway_node || true'
# heaptrack exits once it has finalized the trace; wait for it to go away.
for _ in $(seq 1 40); do
    docker exec "$NAME" pgrep -x heaptrack >/dev/null 2>&1 || break
    sleep 3
done
sleep 3
trace="$(docker exec "$NAME" sh -c 'ls -1 /tmp/heap/gateway* 2>/dev/null | head -1' | tr -d '[:space:]')"
if [ -z "$trace" ]; then
    echo "no heaptrack trace produced; /tmp/heap contents:"
    docker exec "$NAME" ls -la /tmp/heap 2>&1 | head
    docker logs "$NAME" 2>&1 | tail -20; exit 1
fi
echo "   trace: $trace"

echo "== analyze inside container (heaptrack_print) =="
# Two passes: one writes the massif heap-over-time series, one writes the text
# summary (peak consumers / leaked). --print-massif suppresses the text summary,
# so they cannot share a single invocation.
docker exec "$NAME" bash -c "heaptrack_print --print-massif /tmp/heap/massif.out '$trace' >/dev/null 2>&1" || true
docker exec "$NAME" bash -c "heaptrack_print '$trace' > /tmp/heap/summary.txt 2>/dev/null" || true
docker cp "$NAME:/tmp/heap/summary.txt" "$OUT/summary.txt" 2>/dev/null || true
docker cp "$NAME:/tmp/heap/massif.out" "$OUT/massif.out" 2>/dev/null || true
docker cp "$NAME:$trace" "$OUT/" 2>/dev/null || true

echo "== verdict: leak vs cache from wall-clock USS series =="
# Primary signal: wall-clock USS from uss.csv (heaptrack-inflated but on the
# correct time axis).  Secondary check: heaptrack massif held-heap plateau
# (axis = cumulative-allocations, NOT wall-clock seconds).
python3 - "$OUT/uss.csv" "$OUT/massif.out" "$OUT/summary.txt" <<'PY' | tee "$OUT/verdict.txt"
import sys
import re

ussp, massif, summ = sys.argv[1], sys.argv[2], sys.argv[3]


def mib(kib):
    return kib / 1024.0


# ---------------------------------------------------------------------------
# PRIMARY: wall-clock USS slope from uss.csv
# ---------------------------------------------------------------------------
uss_verdict = None
try:
    rows = [r.strip().split(',') for r in open(ussp).read().splitlines()[1:] if ',' in r]
    vals = [(int(t), int(u)) for t, u in rows if t.strip().lstrip('-').isdigit() and u.strip().isdigit()]
    print(f"USS /proc samples: {len(vals)} (heaptrack-inflated: heaptrack shadow memory "
          "is included in gateway address-space USS)")
    if len(vals) >= 4:
        first_t, first_u = vals[0]
        last_t, last_u = vals[-1]
        print(f"USS: {mib(first_u):.1f} MiB (t={first_t}s) -> {mib(last_u):.1f} MiB (t={last_t}s)")
        # Late-window slope: last 40% of samples to avoid startup-burst bias
        late = vals[int(len(vals) * 0.6):]
        if len(late) >= 2:
            span_s = (late[-1][0] - late[0][0]) or 1
            slope_kib_s = (late[-1][1] - late[0][1]) / span_s
            print(f"late-window USS slope: {slope_kib_s * 1024:.1f} B/s "
                  f"over {span_s}s ({len(late)} samples)")
            if slope_kib_s <= 0:
                uss_verdict = "PLATEAU"
                print("VERDICT: PLATEAU -> USS is flat or decreasing in the late window. "
                      "Not a leak (startup-burst has settled).")
            elif slope_kib_s * 1024 < 500:
                uss_verdict = "STABLE"
                print(f"VERDICT: STABLE -> late-window slope {slope_kib_s * 1024:.1f} B/s "
                      "is below 500 B/s. Consistent with bounded cache or heaptrack overhead.")
            else:
                uss_verdict = "GROWING"
                print(f"VERDICT: GROWING -> late-window slope {slope_kib_s * 1024:.1f} B/s "
                      "exceeds 500 B/s. Investigate top allocators below.")
    else:
        print("USS series too short to judge (need >= 4 samples).")
except Exception as exc:
    print(f"USS parse error: {exc}")

# ---------------------------------------------------------------------------
# SECONDARY: heaptrack massif held-heap plateau check
# Axis: cumulative-allocations (heaptrack time units), NOT wall-clock seconds.
# Used only as a corroborating check, not as the primary verdict.
# ---------------------------------------------------------------------------
times, heaps = [], []
try:
    t = None
    for ln in open(massif):
        m = re.match(r'\s*time=(\d+)', ln)
        if m:
            t = int(m.group(1))
        m = re.match(r'\s*mem_heap_B=(\d+)', ln)
        if m and t is not None:
            times.append(t)
            heaps.append(int(m.group(1)))
            t = None
except FileNotFoundError:
    pass

if len(heaps) >= 6:
    n = len(heaps)
    # Use last 40% of snapshots as the late window to match USS logic
    late_h = heaps[int(n * 0.6):]
    late_t = times[int(n * 0.6):]
    span_alloc = (late_t[-1] - late_t[0]) or 1
    # Slope in bytes per cumulative-allocation unit (NOT per second)
    slope_alloc = (late_h[-1] - late_h[0]) / span_alloc
    end_heap = heaps[-1]
    peak_heap = max(heaps)
    print(f"\nheaptrack massif (secondary check, axis=cumulative-allocations):")
    print(f"  peak held-heap: {end_heap / 1024 / 1024:.1f} MiB, "
          f"peak ever: {peak_heap / 1024 / 1024:.1f} MiB, "
          f"snapshots: {n}")
    print(f"  late-window slope: {slope_alloc:.2f} B / cumulative-allocation-unit "
          "(NOT B/s - do not compare to USS slope)")
    if slope_alloc <= 0:
        print("  massif check: held-heap plateau confirmed (late slope <= 0).")
    else:
        print("  massif check: held-heap still growing in late window; see top allocators.")
else:
    print("\nheaptrack massif series too short to check (< 6 snapshots).")

print("\nPeak memory consumers (from heaptrack_print) - the call-sites holding heap:")
try:
    lines = open(summ).read().splitlines()
    shown = 0
    grab = 0
    for ln in lines:
        if re.search(r'peak (memory )?consum|peak contribution', ln, re.I):
            grab = 30
        if grab and ln.strip():
            print("  " + ln[:150])
            shown += 1
            grab -= 1
        if re.search(r'total memory leaked|peak heap memory consumption', ln, re.I):
            print("  " + ln.strip()[:150])
    if not shown:
        print("  (no peak-consumer section parsed; see summary.txt)")
except FileNotFoundError:
    print("  (no summary.txt)")
PY

echo ""
echo "artifacts: $OUT"
