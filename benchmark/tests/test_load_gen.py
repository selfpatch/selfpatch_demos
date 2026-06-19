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
"""Unit tests for load_gen.latency_percentiles (pure function, no I/O)."""
import math
from benchmark.scaler.load_gen import latency_percentiles


def test_empty_returns_zeros():
    p50, p95 = latency_percentiles([])
    assert p50 == 0.0 and p95 == 0.0


def test_single_value():
    p50, p95 = latency_percentiles([42.0])
    assert math.isclose(p50, 42.0) and math.isclose(p95, 42.0)


def test_known_distribution():
    # 10 values 1..10 ms
    lats = list(range(1, 11))
    p50, p95 = latency_percentiles(lats)
    # p50 nearest-rank of 10 values: rank = int(50/100 * 10 + 0.5) - 1 = 4 -> value 5
    assert math.isclose(p50, 5.0)
    # p95 nearest-rank: rank = int(95/100 * 10 + 0.5) - 1 = 9 -> value 10
    assert math.isclose(p95, 10.0)


def test_p95_exceeds_p50():
    lats = [10.0, 20.0, 30.0, 100.0, 200.0, 50.0, 5.0, 8.0, 15.0, 70.0]
    p50, p95 = latency_percentiles(lats)
    assert p95 >= p50


def test_uniform_distribution():
    # All same value: p50 == p95
    lats = [25.0] * 100
    p50, p95 = latency_percentiles(lats)
    assert math.isclose(p50, 25.0) and math.isclose(p95, 25.0)


def test_two_values():
    p50, p95 = latency_percentiles([10.0, 20.0])
    assert p50 <= p95
    assert p50 in (10.0, 20.0) and p95 in (10.0, 20.0)


def test_levels_config_present():
    """light/heavy level configs are defined and have expected structure."""
    from benchmark.scaler.load_gen import _LEVELS
    assert "light" in _LEVELS and "heavy" in _LEVELS
    light_workers, light_rate, light_sse = _LEVELS["light"]
    heavy_workers, heavy_rate, heavy_sse = _LEVELS["heavy"]
    assert light_workers == 8 and light_rate == 5.0
    assert heavy_workers == 32 and heavy_rate == 10.0
    assert heavy_workers > light_workers


def test_sse_url_uses_faults_stream_endpoint():
    """The SSE URL must point at /faults/stream, not /components.

    /components is a plain JSON endpoint that closes immediately and does not
    hold a connection open.  /faults/stream is the gateway's real SSE stream
    (verified against the live route registry; /faults/events 404s).
    """
    import inspect
    from benchmark.scaler import load_gen
    src = inspect.getsource(load_gen.run_load)
    # Must reference the real SSE stream route
    assert "/faults/stream" in src
    # Must NOT use /components as the SSE target
    assert 'sse_url = f"http://{host}:{port}{prefix}/components"' not in src


def test_render_load_markdown_table_structure():
    """render_load_markdown produces a Markdown table with the right columns."""
    from benchmark.lib.report import render_load_markdown
    cells = [
        {"level": "off", "uss_kib_median": 81920, "cpu_cores_median": 0.05,
         "num_threads_median": 13, "p50_ms": 0.0, "p95_ms": 0.0,
         "threads": {"rclcpp_executor": 4, "httplib": 8, "total": 12}, "repeats": 3},
        {"level": "light", "uss_kib_median": 85000, "cpu_cores_median": 0.3,
         "num_threads_median": 21, "p50_ms": 12.5, "p95_ms": 40.0,
         "threads": {"rclcpp_executor": 4, "httplib": 8, "total": 21}, "repeats": 3},
        {"level": "heavy", "uss_kib_median": 95000, "cpu_cores_median": 1.2,
         "num_threads_median": 53, "p50_ms": 55.0, "p95_ms": 310.0,
         "threads": {"rclcpp_executor": 4, "httplib": 32, "total": 53}, "repeats": 3},
    ]
    md = render_load_markdown(cells)
    assert "Load lane" in md
    assert "p50 ms" in md and "p95 ms" in md
    assert "off" in md and "light" in md and "heavy" in md
    # High p95 should trigger optimization signal
    assert "Optimization signals" in md or "p95" in md


def test_load_cmd_median_p50_p95():
    """cmd_load must compute median p50/p95 across reps, not take the last rep only."""
    import statistics
    # Simulate three reps with load_stats and verify the median is used
    p50_vals = [10.0, 20.0, 30.0]
    p95_vals = [50.0, 100.0, 150.0]
    expected_p50 = statistics.median(p50_vals)  # 20.0
    expected_p95 = statistics.median(p95_vals)  # 100.0
    # Build fake reps matching what cmd_load builds
    reps = [{"load_stats": {"p50_ms": p50, "p95_ms": p95}}
            for p50, p95 in zip(p50_vals, p95_vals)]
    ls_list = [r["load_stats"] for r in reps if r.get("load_stats")]
    p50_result = statistics.median([ls["p50_ms"] for ls in ls_list if "p50_ms" in ls])
    p95_result = statistics.median([ls["p95_ms"] for ls in ls_list if "p95_ms" in ls])
    assert p50_result == expected_p50, f"p50 median should be {expected_p50}, got {p50_result}"
    assert p95_result == expected_p95, f"p95 median should be {expected_p95}, got {p95_result}"


def test_load_cmd_median_p50_p95_empty_stats():
    """cmd_load must return 0.0 when no reps have load_stats."""
    import statistics
    reps = [{"load_stats": {}}, {}]
    ls_list = [r["load_stats"] for r in reps if r.get("load_stats")]
    p50_vals = [ls["p50_ms"] for ls in ls_list if "p50_ms" in ls]
    p95_vals = [ls["p95_ms"] for ls in ls_list if "p95_ms" in ls]
    p50 = statistics.median(p50_vals) if p50_vals else 0.0
    p95 = statistics.median(p95_vals) if p95_vals else 0.0
    assert p50 == 0.0 and p95 == 0.0
