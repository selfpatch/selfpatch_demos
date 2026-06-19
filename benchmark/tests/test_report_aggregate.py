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
from benchmark.lib.report import aggregate_cell, scaling_verdict, leak_verdict


def test_aggregate_passthrough_and_median():
    reps = [{"uss_kib": u, "cpu_cores": c, "samples": 100, "label": "x"}
            for u, c in [(100, .5), (110, .6), (105, .55), (108, .52), (102, .58)]]
    out = aggregate_cell(reps)
    assert out["uss_kib_median"] == 105
    assert out["uss_kib_q1"] <= 105 <= out["uss_kib_q3"]
    assert out["samples"] == 100 and out["label"] == "x"   # passthrough, not _median
    assert "samples_median" not in out
    assert out["repeats"] == 5


def test_scaling_verdict_superlinear_from_exponent():
    # A strongly super-linear curve: y ~ x^1.7; with 4 points the CI should be above 1
    v, meta = scaling_verdict([10, 50, 100, 200],
                              [50000, 600000, 2000000, 8000000])
    assert "CONFIRMED" in v and meta["exponent"] > 1.1
    assert meta["ci_lo"] > 1.0


def test_scaling_verdict_linear_not_confirmed():
    # Perfectly linear data: exponent ~ 1.0 with few points -> INDETERMINATE
    v, meta = scaling_verdict([10, 50, 100, 200],
                              [50000, 250000, 500000, 1000000])
    # With only 4 perfectly linear points the CI spans 1: result is INDETERMINATE, not NOT
    assert "INDETERMINATE" in v and abs(meta["exponent"] - 1.0) < 0.1


def test_leak_verdict_suspected_ci_excludes_zero():
    assert "SUSPECTED" in leak_verdict((120, 40, 200), 50 * 1024 * 1024)


def test_leak_verdict_clear_ci_straddles_zero():
    assert "NO LEAK" in leak_verdict((5, -30, 40), 1024)
