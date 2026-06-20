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
from benchmark.lib.report import leak_verdict, render_heap_markdown
from benchmark.lib.leak_parse import HeaptrackSummary


def test_heap_md():
    s = HeaptrackSummary(2 * 1024 * 1024, 80 * 1024 * 1024,
                         [(1024 * 1024, "EntityCache::rebuild()")])
    md = render_heap_markdown(s, (100, 40, 160), "leak: SUSPECTED")
    assert "EntityCache::rebuild" in md and "SUSPECTED" in md and "2.0 MiB" in md


def test_leak_verdict_suspected_wording():
    """The SUSPECTED verdict describes heaptrack's leaked-at-exit total, not a
    'heap grew' growth delta (the value passed is total_leaked_bytes)."""
    v = leak_verdict((100.0, 50.0, 160.0), 3 * 1024 * 1024,
                     warmup_converged=True, has_sites=True)
    assert "SUSPECTED" in v
    assert "leaked" in v and "at exit" in v
    assert "heap grew" not in v


def test_leak_verdict_no_leak_when_ci_straddles_zero():
    v = leak_verdict((0.0, -10.0, 10.0), 0)
    assert "NO LEAK" in v
