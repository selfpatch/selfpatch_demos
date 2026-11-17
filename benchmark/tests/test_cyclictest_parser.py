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

from pathlib import Path

from benchmark.lib import cyclictest

_FIXTURE = Path(__file__).parent / "fixtures" / "cyclictest_h.txt"


def test_parse_real_cyclictest_histogram():
    """Summary lines and the overflow marker of a real `cyclictest -h` run."""
    r = cyclictest.parse_summary(_FIXTURE.read_text())
    assert (r["min_us"], r["avg_us"], r["max_us"]) == (3, 28, 7382)
    assert r["overflow"] == 4
    assert r["samples"] == 9976  # 9972 in the histogram plus 4 overflows
    assert r["p99_us"] == 204
    assert r["p99_lower_bound"] is False
