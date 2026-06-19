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
from benchmark.lib.report import (
    render_footprint_markdown,
    render_scaling_markdown,
    render_sweep_markdown,
    render_heap_markdown,
)
from benchmark.lib.leak_parse import HeaptrackSummary


def _cell():
    return {"label": "default", "uss_kib_median": 81920, "uss_kib_q1": 81000,
            "uss_kib_q3": 82500, "uss_within_iqr_lo_median": 81500,
            "uss_within_iqr_hi_median": 82000, "pss_kib_median": 43000,
            "rss_kib_median": 90000, "cpu_cores_median": 0.42,
            "uss_slope_b_s_median": 12, "uss_slope_lo95_median": -40,
            "uss_slope_hi95_median": 64, "peak_uss_kib_median": 95000,
            "num_threads_median": 13, "repeats": 5, "samples": 270}


def _base_meta():
    return {"cpu_model": "x", "nproc": 8, "mem_total_kb": 0}


def test_markdown_has_uss_and_slope():
    md = render_footprint_markdown([_cell()], _base_meta())
    assert "USS" in md and "slope" in md and "80.0 MiB" in md


def test_footprint_no_warning_when_load_normal():
    meta = {**_base_meta(), "high_host_load": False, "host_load1": 1.0}
    md = render_footprint_markdown([_cell()], meta)
    assert "WARNING" not in md


def test_footprint_warning_when_high_load():
    meta = {**_base_meta(), "high_host_load": True, "host_load1": 10.2, "nproc": 8}
    md = render_footprint_markdown([_cell()], meta)
    assert "WARNING" in md
    assert "load1=10.2" in md
    assert "8 cores" in md


def test_scaling_warning_when_high_load():
    row = {"entity_count": 50, "node_count": 10, "refresh_ms": 1000,
           "uss_kib_median": 40960, "uss_kib_q1": 40000, "uss_kib_q3": 41000,
           "uss_per_entity": 819.2, "cpu_cores_median": 0.3, "repeats": 3}
    fit = {"exponent": 0.5, "r2": 0.9, "ci_lo": 0.3, "ci_hi": 0.7}
    meta = {**_base_meta(), "high_host_load": True, "host_load1": 9.5, "nproc": 8}
    md = render_scaling_markdown([row], "INDETERMINATE", fit, meta=meta)
    assert "WARNING" in md


def test_scaling_no_warning_without_meta():
    row = {"entity_count": 50, "node_count": 10, "refresh_ms": 1000,
           "uss_kib_median": 40960, "uss_kib_q1": 40000, "uss_kib_q3": 41000,
           "uss_per_entity": 819.2, "cpu_cores_median": 0.3, "repeats": 3}
    fit = {"exponent": 0.5, "r2": 0.9, "ci_lo": 0.3, "ci_hi": 0.7}
    md = render_scaling_markdown([row], "INDETERMINATE", fit)
    assert "WARNING" not in md


def test_sweep_warning_when_high_load():
    c = {**_cell(), "label": "default"}
    meta = {**_base_meta(), "high_host_load": True, "host_load1": 12.0, "nproc": 8}
    md = render_sweep_markdown([c], meta=meta)
    assert "WARNING" in md


def test_heap_warning_when_high_load():
    s = HeaptrackSummary(2 * 1024 * 1024, 80 * 1024 * 1024,
                         [(1024 * 1024, "EntityCache::rebuild()")])
    meta = {**_base_meta(), "high_host_load": True, "host_load1": 9.0, "nproc": 8}
    md = render_heap_markdown(s, (100, 40, 160), "leak: SUSPECTED", meta=meta)
    assert "WARNING" in md
    assert "EntityCache::rebuild" in md and "SUSPECTED" in md and "2.0 MiB" in md


def test_heap_no_warning_without_meta():
    s = HeaptrackSummary(2 * 1024 * 1024, 80 * 1024 * 1024,
                         [(1024 * 1024, "EntityCache::rebuild()")])
    md = render_heap_markdown(s, (100, 40, 160), "leak: SUSPECTED")
    assert "EntityCache::rebuild" in md and "SUSPECTED" in md and "2.0 MiB" in md


def test_heap_markdown_discloses_heaptrack_inflation():
    """render_heap_markdown must include a disclosure that USS is heaptrack-inflated."""
    s = HeaptrackSummary(0, 50 * 1024 * 1024, [])
    md = render_heap_markdown(s, (5, -10, 20), "USS slope CI straddles zero -> NO LEAK detected")
    # The disclosure must mention heaptrack inflation so a reader knows a rising
    # slope or SUSPECTED verdict is not solely attributable to the gateway.
    assert "heaptrack" in md.lower() and "inflat" in md.lower()
