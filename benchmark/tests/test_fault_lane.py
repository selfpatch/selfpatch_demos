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
"""Unit tests for fault lane pure functions (no docker required)."""
from benchmark.lib.fault_injector import (
    build_inject_yaml,
    count_rosbag_warns,
    build_fault_rows,
)
from benchmark.lib.report import render_fault_markdown, _fault_optimization_signals


# ---------------------------------------------------------------------------
# build_inject_yaml
# ---------------------------------------------------------------------------

def test_build_inject_yaml_contains_fields():
    yaml_str = build_inject_yaml("bench", "CODE_1", "desc")
    assert "source_id: 'bench'" in yaml_str
    assert "fault_code: 'CODE_1'" in yaml_str
    assert "severity: 3" in yaml_str
    assert "description: 'desc'" in yaml_str


def test_build_inject_yaml_severity_is_critical():
    yaml_str = build_inject_yaml("x", "Y", "z")
    # CRITICAL = 3
    assert "severity: 3" in yaml_str


# ---------------------------------------------------------------------------
# count_rosbag_warns
# ---------------------------------------------------------------------------

def test_no_warns_means_all_got_rosbag():
    log = "some log text without any skip lines"
    assert count_rosbag_warns(log, 4) == 4


def test_one_warn_means_one_missed():
    log = (
        "[WARN] Already recording post-fault data, skipping\n"
        "other log lines\n"
    )
    assert count_rosbag_warns(log, 4) == 3


def test_all_skipped_except_one():
    marker = "Already recording post-fault data, skipping"
    log = "\n".join([marker] * 7)
    # 8 faults, 7 skipped -> 1 got rosbag
    assert count_rosbag_warns(log, 8) == 1


def test_more_skips_than_faults_clamps_to_zero():
    marker = "Already recording post-fault data, skipping"
    log = "\n".join([marker] * 10)
    # 5 faults, 10 skips -> clamped to 0
    assert count_rosbag_warns(log, 5) == 0


def test_no_faults_returns_zero():
    assert count_rosbag_warns("", 0) == 0


# ---------------------------------------------------------------------------
# build_fault_rows
# ---------------------------------------------------------------------------

def _sample_burst_result(peak_kib=512, cpu=0.5, duration=2.0,
                         residual_kib=0.0, recovered=True):
    return {
        "baseline_uss_kib": 10000.0,
        "peak_uss_delta_kib": float(peak_kib),
        "peak_cpu_cores": cpu,
        "capture_duration_s": duration,
        "residual_uss_delta_kib": float(residual_kib),
        "recovered": recovered,
    }


def test_build_fault_rows_basic():
    burst_results = {
        (1, "data"): _sample_burst_result(512, 0.3, 1.5),
        (1, "rosbag"): _sample_burst_result(1024, 0.8, 3.0),
    }
    rosbag_counts = {(1, "data"): 0, (1, "rosbag"): 1}
    rows = build_fault_rows([1], ["data", "rosbag"], burst_results, rosbag_counts)
    assert len(rows) == 2
    data_row = next(r for r in rows if r["mode"] == "data")
    rosbag_row = next(r for r in rows if r["mode"] == "rosbag")
    assert data_row["n"] == 1
    assert abs(data_row["peak_uss_delta_mib"] - 512 / 1024) < 0.001
    assert data_row["rosbag_total"] == 0
    assert rosbag_row["rosbag_total"] == 1
    assert rosbag_row["rosbag_got"] == 1


def test_build_fault_rows_rosbag_total_is_n():
    burst_results = {(4, "rosbag"): _sample_burst_result()}
    rosbag_counts = {(4, "rosbag"): 2}
    rows = build_fault_rows([4], ["rosbag"], burst_results, rosbag_counts)
    assert rows[0]["rosbag_total"] == 4
    assert rows[0]["rosbag_got"] == 2


def test_build_fault_rows_missing_key_gives_zeros():
    # No entry in burst_results -> row uses safe defaults.
    rows = build_fault_rows([8], ["data"], {}, {})
    assert len(rows) == 1
    r = rows[0]
    assert r["peak_uss_delta_mib"] == 0.0
    assert r["capture_duration_s"] == 0.0
    assert r["recovered"] is False


def test_build_fault_rows_all_ns():
    n_list = [1, 2, 4, 8, 16]
    burst_results = {(n, "data"): _sample_burst_result(n * 256) for n in n_list}
    rosbag_counts = {(n, "data"): 0 for n in n_list}
    rows = build_fault_rows(n_list, ["data"], burst_results, rosbag_counts)
    assert len(rows) == len(n_list)
    ns = [r["n"] for r in rows]
    assert ns == n_list


# ---------------------------------------------------------------------------
# render_fault_markdown
# ---------------------------------------------------------------------------

def test_render_fault_markdown_has_required_columns():
    rows = [
        {"n": 1, "mode": "data", "peak_uss_delta_mib": 0.5, "peak_cpu_cores": 0.3,
         "capture_duration_s": 1.5, "residual_mib": 0.0, "recovered": True,
         "rosbag_got": 0, "rosbag_total": 0},
        {"n": 1, "mode": "rosbag", "peak_uss_delta_mib": 1.0, "peak_cpu_cores": 0.6,
         "capture_duration_s": 3.0, "residual_mib": 0.1, "recovered": True,
         "rosbag_got": 1, "rosbag_total": 1},
    ]
    md = render_fault_markdown(rows)
    assert "N faults" in md
    assert "mode" in md
    assert "peak USS delta" in md
    assert "capture duration" in md
    assert "rosbag got/N" in md
    assert "1/1" in md    # rosbag_got/rosbag_total for rosbag mode
    assert "n/a" in md    # no rosbag for data mode


def test_render_fault_markdown_recovered_flag():
    rows = [
        {"n": 2, "mode": "data", "peak_uss_delta_mib": 0.3, "peak_cpu_cores": 0.1,
         "capture_duration_s": 60.0, "residual_mib": 5.0, "recovered": False,
         "rosbag_got": 0, "rosbag_total": 0},
    ]
    md = render_fault_markdown(rows)
    assert "no" in md   # recovered=False


def test_render_fault_markdown_high_load_warning():
    rows = [
        {"n": 1, "mode": "data", "peak_uss_delta_mib": 0.5, "peak_cpu_cores": 0.3,
         "capture_duration_s": 1.5, "residual_mib": 0.0, "recovered": True,
         "rosbag_got": 0, "rosbag_total": 0},
    ]
    meta = {"cpu_model": "x", "nproc": 4, "mem_total_kb": 0,
            "high_host_load": True, "host_load1": 8.0}
    md = render_fault_markdown(rows, meta)
    assert "WARNING" in md


# ---------------------------------------------------------------------------
# _fault_optimization_signals
# ---------------------------------------------------------------------------

def test_signal_capture_grows_with_n():
    rows = [
        {"n": 1, "mode": "data", "capture_duration_s": 1.0,
         "peak_uss_delta_mib": 0.5, "rosbag_got": 0, "rosbag_total": 0},
        {"n": 8, "mode": "data", "capture_duration_s": 10.0,
         "peak_uss_delta_mib": 0.5, "rosbag_got": 0, "rosbag_total": 0},
    ]
    signals = _fault_optimization_signals(rows)
    assert any("capture duration" in s for s in signals)


def test_signal_rosbag_contention():
    rows = [
        {"n": 8, "mode": "rosbag", "capture_duration_s": 5.0,
         "peak_uss_delta_mib": 2.0, "rosbag_got": 1, "rosbag_total": 8},
    ]
    signals = _fault_optimization_signals(rows)
    assert any("rosbag contention" in s for s in signals)


def test_no_signal_when_data_is_flat():
    rows = [
        {"n": 1, "mode": "data", "capture_duration_s": 2.0,
         "peak_uss_delta_mib": 0.5, "rosbag_got": 0, "rosbag_total": 0},
        {"n": 8, "mode": "data", "capture_duration_s": 2.1,
         "peak_uss_delta_mib": 0.5, "rosbag_got": 0, "rosbag_total": 0},
    ]
    signals = _fault_optimization_signals(rows)
    assert not any("capture duration" in s for s in signals)


def test_no_signal_when_all_get_rosbag():
    rows = [
        {"n": 4, "mode": "rosbag", "capture_duration_s": 3.0,
         "peak_uss_delta_mib": 1.0, "rosbag_got": 4, "rosbag_total": 4},
    ]
    signals = _fault_optimization_signals(rows)
    assert not any("rosbag contention" in s for s in signals)


# ---------------------------------------------------------------------------
# single-sample disclosure in render_fault_markdown
# ---------------------------------------------------------------------------

def test_render_fault_markdown_single_sample_disclosure():
    """render_fault_markdown must state that fault cells are single-sample (n=1)."""
    rows = [
        {"n": 1, "mode": "data", "peak_uss_delta_mib": 0.5, "peak_cpu_cores": 0.3,
         "capture_duration_s": 1.5, "residual_mib": 0.0, "recovered": True,
         "rosbag_got": 0, "rosbag_total": 0},
    ]
    md = render_fault_markdown(rows)
    # The output must disclose that values are single-sample point estimates.
    assert "n=1" in md or "single-sample" in md
