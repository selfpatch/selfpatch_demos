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
"""Unit tests for lib/compare.py - the regression tracking logic."""
import json
import pytest
from pathlib import Path
from benchmark.lib.compare import load_baseline, host_matches, extract_metrics, diff


BASELINE = {
    "gateway_sha": "abc123",
    "host": {
        "cpu_model": "Intel(R) Core(TM) i7-10700K CPU @ 3.80GHz",
        "nproc": 16,
        "mem_total_kb": 32824960,
    },
    "footprint": {
        "footprint.uss_kib_median": 90000.0,
        "footprint.cpu_cores_median": 0.2,
        "footprint.num_threads_median": 50.0,
    },
    "scaling": {
        "scaling.exponent": 0.46,
        "scaling.ci_lo": 0.26,
        "scaling.ci_hi": 0.65,
    },
    "load": {
        "load.heavy_uss_kib_median": 35000.0,
        "load.heavy_p95_ms": 2.5,
    },
}

RUN_META_GOOD = {
    "cpu_model": "Intel(R) Core(TM) i7-10700K CPU @ 3.80GHz",
    "nproc": 16,
    "high_host_load": False,
}


# ---------------------------------------------------------------------------
# load_baseline
# ---------------------------------------------------------------------------

def test_load_baseline_reads_json(tmp_path):
    p = tmp_path / "bl.json"
    p.write_text(json.dumps({"host": {"nproc": 8}}))
    bl = load_baseline(p)
    assert bl["host"]["nproc"] == 8


def test_load_baseline_missing_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_baseline(tmp_path / "nonexistent.json")


# ---------------------------------------------------------------------------
# host_matches
# ---------------------------------------------------------------------------

def test_host_matches_identical():
    ok, reason = host_matches(BASELINE, RUN_META_GOOD)
    assert ok
    assert reason == ""


def test_host_matches_cpu_mismatch():
    meta = dict(RUN_META_GOOD, cpu_model="AMD Ryzen 9 5900X")
    ok, reason = host_matches(BASELINE, meta)
    assert not ok
    assert "cpu_model" in reason


def test_host_matches_nproc_mismatch():
    meta = dict(RUN_META_GOOD, nproc=8)
    ok, reason = host_matches(BASELINE, meta)
    assert not ok
    assert "nproc" in reason


# ---------------------------------------------------------------------------
# extract_metrics
# ---------------------------------------------------------------------------

def test_extract_footprint():
    summary = {
        "status": "ok",
        "cell": {
            "uss_kib_median": 95000.0,
            "cpu_cores_median": 0.21,
            "num_threads_median": 52.0,
        },
    }
    m = extract_metrics("footprint", summary)
    assert m["footprint.uss_kib_median"] == 95000.0
    assert m["footprint.cpu_cores_median"] == 0.21
    assert m["footprint.num_threads_median"] == 52.0


def test_extract_scaling():
    summary = {
        "fit": {"exponent": 0.50, "ci_lo": 0.30, "ci_hi": 0.70, "r2": 0.95},
    }
    m = extract_metrics("scaling", summary)
    assert m["scaling.exponent"] == 0.50
    assert m["scaling.ci_lo"] == 0.30


def test_extract_load_heavy():
    summary = {
        "cells": [
            {"level": "off", "uss_kib_median": 28000.0},
            {"level": "light", "uss_kib_median": 30000.0},
            {"level": "heavy", "uss_kib_median": 36000.0, "p95_ms": 3.0},
        ]
    }
    m = extract_metrics("load", summary)
    assert m["load.heavy_uss_kib_median"] == 36000.0
    assert m["load.heavy_p95_ms"] == 3.0


def test_extract_load_no_heavy():
    summary = {"cells": [{"level": "off", "uss_kib_median": 28000.0}]}
    m = extract_metrics("load", summary)
    assert "load.heavy_uss_kib_median" not in m


def test_extract_fault_empty():
    """Fault lane is informational only - extract returns empty dict."""
    m = extract_metrics("fault", {"status": "ok", "rows": []})
    assert m == {}


def test_extract_unknown_lane():
    m = extract_metrics("unknown_lane", {"status": "ok"})
    assert m == {}


# ---------------------------------------------------------------------------
# diff
# ---------------------------------------------------------------------------

def test_diff_ok_no_regression():
    new_metrics = {
        "footprint.uss_kib_median": 91000.0,  # +1.1% -> OK
        "footprint.cpu_cores_median": 0.205,   # +2.5% -> OK
        "footprint.num_threads_median": 51.0,  # +2% -> OK
    }
    rows = diff(BASELINE, "footprint", new_metrics)
    verdicts = {r["metric"]: r["verdict"] for r in rows}
    assert all(v == "OK" for v in verdicts.values()), verdicts


def test_diff_warn_uss():
    new_metrics = {
        "footprint.uss_kib_median": 95400.0,  # +6% -> WARN (>5%)
        "footprint.cpu_cores_median": 0.2,
        "footprint.num_threads_median": 50.0,
    }
    rows = diff(BASELINE, "footprint", new_metrics)
    verdicts = {r["metric"]: r["verdict"] for r in rows}
    assert verdicts["footprint.uss_kib_median"] == "WARN"


def test_diff_regression_uss():
    new_metrics = {
        "footprint.uss_kib_median": 99001.0,  # +10.0% -> REGRESSION (>=10%)
        "footprint.cpu_cores_median": 0.2,
        "footprint.num_threads_median": 50.0,
    }
    rows = diff(BASELINE, "footprint", new_metrics)
    verdicts = {r["metric"]: r["verdict"] for r in rows}
    assert verdicts["footprint.uss_kib_median"] == "REGRESSION"


def test_diff_scaling_exponent_ci_lo_above_1_regression():
    """Scaling exponent CI lower bound crossing 1.0 is a REGRESSION."""
    new_metrics = {
        "scaling.exponent": 1.2,
        "scaling.ci_lo": 1.05,  # ci_lo > 1.0 -> REGRESSION
        "scaling.ci_hi": 1.4,
    }
    rows = diff(BASELINE, "scaling", new_metrics)
    exp_row = next(r for r in rows if r["metric"] == "scaling.exponent")
    assert exp_row["verdict"] == "REGRESSION"


def test_diff_scaling_exponent_ci_lo_below_1_ok():
    """If ci_lo <= 1.0, scaling exponent is not a regression even if it grew."""
    new_metrics = {
        "scaling.exponent": 0.55,
        "scaling.ci_lo": 0.35,
        "scaling.ci_hi": 0.75,
    }
    rows = diff(BASELINE, "scaling", new_metrics)
    exp_row = next(r for r in rows if r["metric"] == "scaling.exponent")
    assert exp_row["verdict"] == "OK"


def test_diff_missing_baseline_metric_na():
    """Metric absent in baseline returns N/A verdict, not an error."""
    new_metrics = {"footprint.uss_kib_median": 95000.0}
    baseline_no_footprint = dict(BASELINE, footprint={})
    rows = diff(baseline_no_footprint, "footprint", new_metrics)
    assert rows[0]["verdict"] == "N/A"


def test_diff_missing_new_metric_na():
    """Metric absent in new run returns N/A."""
    new_metrics = {"footprint.uss_kib_median": None}
    rows = diff(BASELINE, "footprint", new_metrics)
    assert rows[0]["verdict"] == "N/A"


def test_diff_delta_pct_computed_correctly():
    new_metrics = {
        "footprint.uss_kib_median": 99000.0,  # from 90000 -> +10%
        "footprint.cpu_cores_median": 0.2,
        "footprint.num_threads_median": 50.0,
    }
    rows = diff(BASELINE, "footprint", new_metrics)
    uss_row = next(r for r in rows if r["metric"] == "footprint.uss_kib_median")
    assert abs(uss_row["delta_pct"] - 10.0) < 0.01
