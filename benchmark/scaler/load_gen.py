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
"""In-container HTTP load generator.

Run as::

    python3 -m benchmark.scaler.load_gen \\
        --level {light,heavy} \\
        --duration S \\
        --base-url http://localhost:8080/api/v1 \\
        --out /tmp/load_stats.json

Levels:

- light: 8 worker threads, each issuing GET requests at 5 req/s to
  /components, /apps, /areas (round-robin).
- heavy: 32 concurrent request workers at 10 req/s cycling through
  /components, /apps, /areas, and the first entity found under /components
  (data endpoint).  Additionally holds 4 real SSE streams open against the
  gateway fault event endpoint (/faults/stream) for the duration.  If that
  endpoint is unavailable the SSE workers fall back to no-op so the
  measurement still runs with 32 request workers.

The module must be stdlib-only (no httpx, no requests) because it runs
INSIDE the gateway container where only the Python standard library is
guaranteed.  Uses http.client for HTTP/1.1 and urllib for SSE streams.

Output JSON::

    {
        "level": "light",
        "duration_s": 30,
        "workers": 8,
        "request_count": 1200,
        "p50_ms": 12.3,
        "p95_ms": 45.1,
        "error_count": 0
    }
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from http.client import HTTPConnection
from urllib.parse import urlparse
from urllib.request import urlopen


# Level configuration: (workers, rate_per_s, sse_streams)
_LEVELS = {
    "light": (8, 5.0, 0),
    "heavy": (32, 10.0, 4),
}

_REST_PATHS = ["/components", "/apps", "/areas"]


def latency_percentiles(latencies_ms: list) -> tuple:
    """Return (p50, p95) from a list of latency values in milliseconds.

    Pure function - no I/O.  Returns (0.0, 0.0) for an empty list.
    """
    if not latencies_ms:
        return 0.0, 0.0
    sorted_lats = sorted(latencies_ms)
    n = len(sorted_lats)

    def _percentile(p: float) -> float:
        # Nearest-rank method
        rank = max(0, int(p / 100.0 * n + 0.5) - 1)
        return float(sorted_lats[min(rank, n - 1)])

    return _percentile(50), _percentile(95)


def _parse_base_url(base_url: str):
    """Return (host, port, path_prefix) from base_url."""
    parsed = urlparse(base_url)
    host = parsed.hostname or "localhost"
    port = parsed.port or 80
    prefix = parsed.path.rstrip("/")
    return host, port, prefix


def _worker(host: str, port: int, prefix: str, paths: list,
            rate_per_s: float, stop_event: threading.Event,
            latencies: list, errors_ref: list, lock: threading.Lock) -> None:
    """Worker thread: issue GET requests at rate_per_s until stop_event is set."""
    interval = 1.0 / rate_per_s if rate_per_s > 0 else 1.0
    idx = 0
    while not stop_event.is_set():
        path = prefix + paths[idx % len(paths)]
        idx += 1
        t0 = time.monotonic()
        try:
            conn = HTTPConnection(host, port, timeout=5)
            conn.request("GET", path)
            resp = conn.getresponse()
            resp.read()
            conn.close()
            elapsed_ms = (time.monotonic() - t0) * 1000.0
            with lock:
                latencies.append(elapsed_ms)
        except Exception:
            with lock:
                errors_ref[0] += 1
        # Rate-limit: sleep for the remainder of the interval
        elapsed = time.monotonic() - t0
        sleep_for = interval - elapsed
        if sleep_for > 0:
            stop_event.wait(timeout=sleep_for)


def _sse_worker(url: str, stop_event: threading.Event) -> None:
    """Hold an SSE connection open until stop_event is set."""
    try:
        with urlopen(url, timeout=None) as resp:  # type: ignore[arg-type]
            while not stop_event.is_set():
                line = resp.readline()
                if not line:
                    break
    except Exception:
        pass


def run_load(base_url: str, level: str, duration_s: float) -> dict:
    """Run load for duration_s seconds and return stats dict."""
    workers, rate_per_s, sse_count = _LEVELS.get(level, _LEVELS["light"])
    host, port, prefix = _parse_base_url(base_url)

    # For heavy level, try to discover one data endpoint to add to rotation.
    rest_paths = list(_REST_PATHS)
    if level == "heavy":
        try:
            conn = HTTPConnection(host, port, timeout=3)
            conn.request("GET", prefix + "/components")
            resp = conn.getresponse()
            body = resp.read().decode("utf-8", errors="replace")
            conn.close()
            data = json.loads(body)
            items = data.get("items", [])
            if items:
                first_id = items[0].get("id", "")
                if first_id:
                    rest_paths.append(f"/components/{first_id}/data")
        except Exception:
            pass

    latencies: list = []
    errors_ref = [0]
    lock = threading.Lock()
    stop_event = threading.Event()

    threads = []
    for _ in range(workers):
        t = threading.Thread(
            target=_worker,
            args=(host, port, prefix, rest_paths, rate_per_s,
                  stop_event, latencies, errors_ref, lock),
            daemon=True,
        )
        t.start()
        threads.append(t)

    # SSE streams (heavy only) - held open for the duration against the real
    # gateway fault event stream.  /faults/stream is a server-sent-events
    # endpoint that keeps the connection alive; /components is a plain JSON
    # response that closes immediately and would not add connection-hold
    # pressure.  Fall back silently if the endpoint is unavailable.
    sse_threads = []
    if sse_count > 0:
        sse_url = f"http://{host}:{port}{prefix}/faults/stream"
        for _ in range(sse_count):
            st = threading.Thread(
                target=_sse_worker,
                args=(sse_url, stop_event),
                daemon=True,
            )
            st.start()
            sse_threads.append(st)

    time.sleep(duration_s)
    stop_event.set()

    for t in threads:
        t.join(timeout=5)
    for st in sse_threads:
        st.join(timeout=5)

    p50, p95 = latency_percentiles(latencies)
    return {
        "level": level,
        "duration_s": duration_s,
        "workers": workers,
        "request_count": len(latencies),
        "p50_ms": round(p50, 2),
        "p95_ms": round(p95, 2),
        "error_count": errors_ref[0],
    }


def main() -> None:
    ap = argparse.ArgumentParser(description="In-container HTTP load generator")
    ap.add_argument("--level", choices=["light", "heavy"], default="light")
    ap.add_argument("--duration", type=float, default=30.0,
                    help="Duration in seconds")
    ap.add_argument("--base-url", default="http://localhost:8080/api/v1",
                    help="Gateway API base URL (reachable from inside the container)")
    ap.add_argument("--out", default="/tmp/load_stats.json",
                    help="Path to write the JSON stats output")
    args = ap.parse_args()

    stats = run_load(args.base_url, args.level, args.duration)
    with open(args.out, "w") as fh:
        json.dump(stats, fh, indent=2)
    print(json.dumps(stats, indent=2))


if __name__ == "__main__":
    main()
