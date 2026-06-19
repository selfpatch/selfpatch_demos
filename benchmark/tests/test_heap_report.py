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
from benchmark.lib.report import render_heap_markdown
from benchmark.lib.leak_parse import HeaptrackSummary


def test_heap_md():
    s = HeaptrackSummary(2 * 1024 * 1024, 80 * 1024 * 1024,
                         [(1024 * 1024, "EntityCache::rebuild()")])
    md = render_heap_markdown(s, (100, 40, 160), "leak: SUSPECTED")
    assert "EntityCache::rebuild" in md and "SUSPECTED" in md and "2.0 MiB" in md
