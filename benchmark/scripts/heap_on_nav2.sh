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
# Heap-on-Nav2: leak-vs-cache for the gateway attached to the real Nav2 stack
# (turtlebot3 demo). Two phases, because heaptrack's own shadow memory lives in
# the gateway address space and inflates /proc USS - a single long heaptrack run
# therefore reports FALSE USS growth (measured: tracked heap ~15 MiB while USS
# climbed 34 MiB under heaptrack, yet the same gateway USS plateaus without it):
#
#   Phase A (PRIMARY, no heaptrack): sample /proc USS over the full window and fit
#     an OLS slope with a 95% CI -> the authoritative leak-vs-plateau verdict,
#     free of any heaptrack inflation.
#   Phase B (ATTRIBUTION, heaptrack, SHORT): run the gateway under heaptrack for a
#     short window so the trace stays small enough for heaptrack_print to actually
#     summarize (a 25-min trace OOMs heaptrack_print), and report the leaked total
#     + top allocation call-sites. Used to localize growth when Phase A flags it.
#
# Usage: heap_on_nav2.sh [USS_DURATION_SEC] [WARMUP_SEC] [ATTR_DURATION_SEC]
set -eo pipefail

USS_DURATION="${1:-1500}"       # Phase A measurement window (default 25 min)
WARMUP="${2:-120}"              # settle Nav2 + gateway before each measurement
ATTR_DURATION="${3:-300}"       # Phase B heaptrack window (short -> small trace)
BASE_IMAGE="${BASE_IMAGE:-bench_fp-turtlebot3-demo-ci:latest}"
HEAP_IMAGE="medkit-heap-nav2:local"
SAMPLE_EVERY=30
HERE="$(cd "$(dirname "$0")/.." && pwd)"   # benchmark/
OUT="${HERE}/results/heap_nav2_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

NAME_A="heap_nav2_base"
NAME_B="heap_nav2_ht"
LAUNCH='mkdir -p /var/lib/ros2_medkit/rosbags /tmp/heap && \
    source /opt/ros/jazzy/setup.bash && source /root/demo_ws/install/setup.bash && \
    export TURTLEBOT3_MODEL=burger && \
    ros2 launch turtlebot3_medkit_demo demo.launch.py headless:=true'

cleanup() { docker rm -f "$NAME_A" "$NAME_B" >/dev/null 2>&1 || true; }
trap cleanup EXIT

wait_ready() {  # $1=container name; returns 0 once /health answers 200
    for _ in $(seq 1 60); do
        if docker exec "$1" curl -fs -o /dev/null \
                http://localhost:8080/api/v1/health 2>/dev/null; then
            return 0
        fi
        sleep 5
    done
    return 1
}

resolve_gpid() {  # $1=container name; echoes the gateway_node pid
    # comm is exactly "gateway_node"; -x avoids matching the heaptrack wrapper.
    docker exec "$1" bash -c 'pgrep -x gateway_node | head -1' 2>/dev/null \
        | tr -d '[:space:]'
}

sample_uss() {  # $1=container $2=pid $3=duration $4=outfile
    local elapsed=0 uss
    echo "t_s,uss_kib" > "$4"
    while [ "$elapsed" -lt "$3" ]; do
        uss="$(docker exec "$1" sh -c \
            "awk '/^Private/{s+=\$2} END{print s}' /proc/$2/smaps_rollup 2>/dev/null \
             || awk '/^Private/{s+=\$2} END{print s}' /proc/$2/smaps 2>/dev/null" \
            2>/dev/null | tr -d '[:space:]')" || uss=""
        echo "${elapsed},${uss:-NA}" >> "$4"
        sleep "$SAMPLE_EVERY"; elapsed=$((elapsed + SAMPLE_EVERY))
    done
}

# ---------------------------------------------------------------------------
# Phase A: PRIMARY verdict - gateway WITHOUT heaptrack (no shadow inflation)
# ---------------------------------------------------------------------------
echo "== Phase A: gateway WITHOUT heaptrack (clean USS), domain 30 =="
docker rm -f "$NAME_A" >/dev/null 2>&1 || true
docker run -d --name "$NAME_A" --shm-size=1g -e ROS_DOMAIN_ID=30 \
    -e TURTLEBOT3_MODEL=burger "$BASE_IMAGE" bash -c "$LAUNCH" >/dev/null
echo "   waiting for gateway HTTP ready ..."
wait_ready "$NAME_A" || {
    echo "Phase A: gateway never became ready"; docker logs "$NAME_A" 2>&1 | tail -30
    exit 1
}
echo "   ready; warmup ${WARMUP}s (let Nav2 + caches settle) ..."
sleep "$WARMUP"
GPID_A="$(resolve_gpid "$NAME_A")"
[ -n "$GPID_A" ] || { echo "Phase A: could not resolve gateway_node pid"; exit 1; }
echo "   gateway pid=${GPID_A}; measuring USS ${USS_DURATION}s every ${SAMPLE_EVERY}s"
sample_uss "$NAME_A" "$GPID_A" "$USS_DURATION" "$OUT/uss.csv"
docker rm -f "$NAME_A" >/dev/null 2>&1 || true

# ---------------------------------------------------------------------------
# Phase B: ATTRIBUTION - gateway UNDER heaptrack, SHORT window -> call-sites
# ---------------------------------------------------------------------------
echo "== Phase B: build heaptrack image (gateway with symbols + heaptrack) =="
docker build -f "${HERE}/profiles/Dockerfile.benchmark-demo" \
    --build-arg "BASE=${BASE_IMAGE}" -t "$HEAP_IMAGE" "${HERE}/profiles" \
    >"$OUT/build.log" 2>&1 \
    || { echo "build failed; see $OUT/build.log"; tail -20 "$OUT/build.log"; exit 1; }
echo "== Phase B: launch gateway(heaptrack), domain 30, ${ATTR_DURATION}s window =="
docker rm -f "$NAME_B" >/dev/null 2>&1 || true
docker run -d --name "$NAME_B" --shm-size=1g -e ROS_DOMAIN_ID=30 \
    -e TURTLEBOT3_MODEL=burger "$HEAP_IMAGE" bash -c "$LAUNCH" >/dev/null
wait_ready "$NAME_B" || {
    echo "Phase B: gateway never became ready under heaptrack"
    docker logs "$NAME_B" 2>&1 | tail -30; exit 1
}
echo "   ready; warmup ${WARMUP}s ..."
sleep "$WARMUP"
echo "   accumulating allocations ${ATTR_DURATION}s ..."
sleep "$ATTR_DURATION"
echo "   stop gateway gracefully so heaptrack finalizes ..."
# SIGINT ONLY the gateway child (comm=gateway_node), not the heaptrack wrapper,
# so heaptrack observes the exit and writes its trace instead of dying mid-finalize.
docker exec "$NAME_B" bash -c 'pkill -INT -x gateway_node || true'
for _ in $(seq 1 40); do
    docker exec "$NAME_B" pgrep -x heaptrack >/dev/null 2>&1 || break
    sleep 3
done
sleep 3
trace="$(docker exec "$NAME_B" sh -c 'ls -1 /tmp/heap/gateway* 2>/dev/null | head -1' \
    | tr -d '[:space:]')"
if [ -n "$trace" ]; then
    echo "   trace: $trace ; heaptrack_print (errors -> heaptrack_print.log) ..."
    docker exec "$NAME_B" bash -c "heaptrack_print '$trace'" \
        >"$OUT/summary.txt" 2>"$OUT/heaptrack_print.log" || true
    docker cp "$NAME_B:$trace" "$OUT/" >/dev/null 2>&1 || true
    if ! grep -q 'total memory leaked' "$OUT/summary.txt" 2>/dev/null; then
        echo "   WARNING: heaptrack_print produced no summary (see "
        echo "            $OUT/heaptrack_print.log); attribution unavailable."
    fi
else
    echo "   WARNING: no heaptrack trace produced; attribution unavailable."
    docker logs "$NAME_B" 2>&1 | tail -20
fi
docker rm -f "$NAME_B" >/dev/null 2>&1 || true

# ---------------------------------------------------------------------------
# Verdict: Phase A USS slope (primary) + Phase B call-site attribution
# ---------------------------------------------------------------------------
echo "== verdict =="
# PYTHONPATH points at the repo root (benchmark/.. ) so the heredoc reuses the
# harness's OLS slope+CI and heaptrack parsers instead of duplicating them.
PYTHONPATH="${HERE}/.." python3 - "$OUT/uss.csv" "$OUT/summary.txt" \
    <<'PY' | tee "$OUT/verdict.txt"
import sys

from benchmark.lib.metrics import slope_ci95
from benchmark.lib.leak_parse import parse_heaptrack_summary

ussp, summ = sys.argv[1], sys.argv[2]


def mib_from_kib(kib):
    return kib / 1024.0


# PRIMARY: clean wall-clock USS slope (Phase A, no heaptrack inflation).
print("PRIMARY (no heaptrack): gateway /proc USS over time on real Nav2")
try:
    rows = [r.strip().split(',') for r in open(ussp).read().splitlines()[1:]
            if ',' in r]
    vals = [(int(t), int(u)) for t, u in rows
            if t.strip().lstrip('-').isdigit() and u.strip().isdigit()]
    print(f"  USS samples: {len(vals)}")
    if len(vals) >= 4:
        print(f"  USS: {mib_from_kib(vals[0][1]):.1f} MiB (t={vals[0][0]}s) -> "
              f"{mib_from_kib(vals[-1][1]):.1f} MiB (t={vals[-1][0]}s)")
        # OLS fit + 95% CI over the late window (last 40%) so a single noisy
        # endpoint cannot decide leak-vs-cache.
        late = vals[int(len(vals) * 0.6):]
        xs = [float(t) for t, _u in late]
        ys = [float(u) * 1024 for _t, u in late]  # bytes
        slope, lo, hi = slope_ci95(xs, ys)
        span = (late[-1][0] - late[0][0]) or 1
        print(f"  late-window OLS slope: {slope:.1f} B/s 95% CI [{lo:.1f},{hi:.1f}] "
              f"over {span}s ({len(late)} samples)")
        if hi <= 0:
            print("  VERDICT: PLATEAU -> USS slope CI entirely <= 0; not a leak.")
        elif lo > 0 and slope >= 500:
            print(f"  VERDICT: GROWING -> slope {slope:.1f} B/s, CI excludes zero "
                  "and >= 500 B/s; real growth - see attribution below.")
        else:
            print(f"  VERDICT: STABLE -> slope {slope:.1f} B/s, CI [{lo:.1f},{hi:.1f}] "
                  "straddles zero or below 500 B/s; bounded cache, not a leak.")
    else:
        print("  USS series too short to judge (need >= 4 samples).")
except Exception as exc:
    print(f"  USS parse error: {exc}")

# ATTRIBUTION: heaptrack top allocation call-sites (Phase B, short trace).
print("\nATTRIBUTION (heaptrack, short run): top allocation call-sites")
try:
    text = open(summ).read()
    if 'total memory leaked' in text:
        s = parse_heaptrack_summary(text)
        print(f"  total leaked at exit: {s.total_leaked_bytes / 1024 / 1024:.1f} MiB")
        print(f"  peak heap: {s.peak_heap_bytes / 1024 / 1024:.1f} MiB")
        if s.top_sites:
            for b, site in s.top_sites[:10]:
                print(f"  - {b / 1024 / 1024:.2f} MiB :: {site}")
        else:
            print("  no per-call-site leaks attributed (clean shutdown).")
    else:
        print("  (heaptrack summary unavailable - see heaptrack_print.log)")
except Exception as exc:
    print(f"  attribution parse error: {exc}")
PY

echo ""
echo "artifacts: $OUT"
