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
"""Unit tests for the churn-gate verdict + report (pure, no docker)."""
from benchmark.lib.report import churn_verdict, render_churn_markdown


def _cell(slope, lo, hi, label="x"):
    return {"label": label, "uss_kib": 50000,
            "uss_slope_b_s": slope, "uss_slope_lo95": lo, "uss_slope_hi95": hi}


def test_churn_verdict_detects_leak():
    """Churn slope CI lower bound far above static -> LEAK (the bug present)."""
    static = _cell(759, -294, 1811)
    churn = _cell(44416, 43133, 45699)
    is_leak, msg = churn_verdict(static, churn)
    assert is_leak is True
    assert "LEAK" in msg


def test_churn_verdict_pass_when_churn_matches_static():
    """After a fix, churn slope falls to the static (flat) level -> PASS."""
    static = _cell(759, -294, 1811)
    churn = _cell(800, -300, 1900)
    is_leak, msg = churn_verdict(static, churn)
    assert is_leak is False
    assert "PASS" in msg


def test_churn_verdict_absolute_floor_suppresses_noise():
    """A tiny churn slope above a very flat static must not flag (noise floor)."""
    static = _cell(50, -50, 100)        # extremely flat control
    churn = _cell(1500, 1200, 1800)     # below the 2000 B/s floor
    is_leak, _ = churn_verdict(static, churn)
    assert is_leak is False


def test_churn_verdict_relative_to_noisy_static():
    """When the static control is itself noisy, churn must clear its CI to flag."""
    static = _cell(3000, 1000, 5000)    # noisy control
    churn = _cell(4000, 3000, 5000)     # within static CI upper bound
    is_leak, _ = churn_verdict(static, churn)
    assert is_leak is False


def test_render_churn_markdown_structure():
    static = _cell(700, -300, 1800, label="static")
    churn = _cell(44000, 43000, 45000, label="churn")
    _, verdict = churn_verdict(static, churn)
    md = render_churn_markdown(static, churn, verdict)
    assert "Churn leak gate" in md
    assert "static" in md and "churn" in md
    assert "Verdict" in md
