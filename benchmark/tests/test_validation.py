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
from benchmark.lib.report import validate_synthetic


def _rows(*entity_counts):
    """Build minimal synth_rows with plausible uss_per_entity values."""
    return [{"entity_count": ec, "uss_per_entity": ec * 100} for ec in entity_counts]


def test_diverges():
    # synth N=20 closest to demo N=22; ratio = 2000/6000 ~ 0.33 -> DIVERGES
    rows = _rows(20)
    assert "DIVERGES" in validate_synthetic(6000, rows, 22)


def test_consistent():
    # synth N=20 closest to demo N=22; ratio = 5700/6000 ~ 0.95 -> consistent
    rows = [{"entity_count": 20, "uss_per_entity": 5700}]
    assert "consistent" in validate_synthetic(6000, rows, 22).lower()


def test_picks_closest_row():
    # demo N=22; synth has N=10 and N=25; N=25 is closer
    rows = [{"entity_count": 10, "uss_per_entity": 3000},
            {"entity_count": 25, "uss_per_entity": 5700}]
    result = validate_synthetic(6000, rows, 22)
    assert "25" in result and "consistent" in result.lower()


def test_no_close_row_reports_gap():
    # demo N=22; closest synth N=100 -> more than 50% away
    rows = [{"entity_count": 100, "uss_per_entity": 5000}]
    result = validate_synthetic(6000, rows, 22)
    assert "no synthetic point" in result


def test_empty_rows():
    assert "INCONCLUSIVE" in validate_synthetic(6000, [], 22)


def test_zero_demo_upe():
    rows = _rows(20)
    assert "INCONCLUSIVE" in validate_synthetic(0, rows, 22)


def test_entity_counts_in_output():
    rows = [{"entity_count": 25, "uss_per_entity": 5700}]
    result = validate_synthetic(6000, rows, 22)
    assert "25" in result and "22" in result
