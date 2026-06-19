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
from pathlib import Path
from benchmark.lib.leak_parse import parse_heaptrack_summary, parse_memcheck_summary

FIX = Path(__file__).parent / "fixtures"


def test_heaptrack():
    s = parse_heaptrack_summary((FIX / "heaptrack_print.txt").read_text())
    assert s.total_leaked_bytes == int(2.50 * 1024 * 1024)
    assert s.peak_heap_bytes == int(84.30 * 1024 * 1024)
    assert s.top_sites[0][0] == int(1.20 * 1024 * 1024)
    assert "EntityCache::rebuild" in s.top_sites[0][1]


def test_memcheck():
    s = parse_memcheck_summary((FIX / "memcheck.txt").read_text())
    assert s.definitely_lost_bytes == 1024 and s.definitely_lost_blocks == 4
    assert s.indirectly_lost_bytes == 2048
