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
from benchmark.lib.sampler import parse_smaps_rollup, parse_status, parse_stat_ticks

FIX = Path(__file__).parent / "fixtures"


def test_smaps_uss_private_only():
    uss, pss, rss = parse_smaps_rollup((FIX / "smaps_rollup.txt").read_text())
    assert uss == 20000 and pss == 42000 and rss == 81920  # Pss_Anon ignored


def test_status_vmrss_threads():
    assert parse_status((FIX / "status.txt").read_text()) == (81920, 13)


def test_stat_ticks_sum():
    assert parse_stat_ticks((FIX / "stat.txt").read_text()) == 660


def test_stat_space_in_comm():
    weird = "4242 (gate way) S 1 2 3 0 -1 0 0 0 0 0 540 120 " + "0 " * 40
    assert parse_stat_ticks(weird) == 660
