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
"""Parse heaptrack_print and valgrind memcheck summaries (pure)."""
from __future__ import annotations
import re
from dataclasses import dataclass

_UNIT = {"K": 1024, "M": 1024 ** 2, "G": 1024 ** 3}


@dataclass
class HeaptrackSummary:
    total_leaked_bytes: int
    peak_heap_bytes: int
    top_sites: list


@dataclass
class MemcheckSummary:
    definitely_lost_bytes: int
    indirectly_lost_bytes: int
    definitely_lost_blocks: int


def _bytes(token):
    m = re.match(r"([\d.]+)\s*([KMG]?)", token)
    if m is None:
        return 0
    return int(float(m.group(1)) * _UNIT.get(m.group(2), 1))


def parse_heaptrack_summary(text):
    total = peak = 0
    sites = []
    lines = text.splitlines()
    for i, line in enumerate(lines):
        s = line.strip()
        if s.startswith("total memory leaked:"):
            total = _bytes(s.split(":")[1].strip())
        elif s.startswith("peak heap memory consumption:"):
            peak = _bytes(s.split(":")[1].strip())
        else:
            m = re.match(r"([\d.]+[KMG]?) leaked over .* calls from", s)
            if m and i + 1 < len(lines):
                sites.append((_bytes(m.group(1)), lines[i + 1].strip()))
    return HeaptrackSummary(total, peak, sites)


def parse_memcheck_summary(text):
    def grab(label):
        m = re.search(label + r":\s*([\d,]+) bytes in ([\d,]+) blocks", text)
        return (int(m.group(1).replace(",", "")), int(m.group(2).replace(",", ""))) if m else (0, 0)
    db, dblk = grab("definitely lost")
    ib, _ = grab("indirectly lost")
    return MemcheckSummary(db, ib, dblk)
