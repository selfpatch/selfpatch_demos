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
import json
from unittest.mock import patch

from benchmark.lib.runner import summarize_window, count_in
from benchmark.lib.metrics import Sample


def test_summary_keys():
    s = [Sample(float(i), 1000 + i, 800, 1200, i * 100, 13, 0.0) for i in range(10)]
    out = summarize_window(s, 100)
    for k in ("uss_kib", "pss_kib", "rss_kib", "cpu_cores", "peak_uss_kib",
              "num_threads", "uss_within_iqr_lo", "uss_slope_lo95",
              "samples", "samples_total"):
        assert k in out, f"missing key: {k}"
    # samples = steady window length (last 1/3 of 10 = 3)
    # samples_total = total sample count
    assert out["samples"] == 3
    assert out["samples_total"] == 10
    assert out["cpu_cores"] >= 0.0


def test_count_in_all_probes_fail_returns_none():
    """A5: count_in returns None when every endpoint probe raises an exception."""
    def raise_always(*_args, **_kwargs):
        raise ConnectionError("no route to host")

    with patch("benchmark.lib.runner._curl", side_effect=raise_always):
        result = count_in("c", 8080, "/api/v1")
    assert result is None


def test_count_in_partial_failure_returns_sum():
    """A5: count_in returns the partial sum when at least one probe succeeds."""
    responses = {
        "/components": (200, json.dumps({"items": [1, 2, 3]})),
        "/apps": (200, json.dumps({"items": [4, 5]})),
    }

    def fake_curl(_container, _port, _api_base, path):
        if path in responses:
            return responses[path]
        raise ConnectionError("endpoint missing")

    with patch("benchmark.lib.runner._curl", side_effect=fake_curl):
        result = count_in("c", 8080, "/api/v1")
    assert result == 5
