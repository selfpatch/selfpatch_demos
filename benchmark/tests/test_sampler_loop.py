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
from benchmark.lib.sampler import sample_once, per_sample_cores
from benchmark.lib.metrics import Sample

FIX = Path(__file__).parent / "fixtures"


def _read(pid, fname):
    return (FIX / {"smaps_rollup": "smaps_rollup.txt", "status": "status.txt",
                   "stat": "stat.txt"}[fname]).read_text()


def test_sample_once_fields():
    s = sample_once(_read, 4242, 0.7)
    assert s.uss_kib == 20000 and s.pss_kib == 42000 and s.rss_kib == 81920
    assert s.total_ticks == 660 and s.num_threads == 13 and s.host_load1 == 0.7


def test_per_sample_cores_between_consecutive():
    a = Sample(0.0, 1, 1, 1, 0, 1, 0.0)
    b = Sample(1.0, 2, 2, 2, 100, 1, 0.0)
    c = Sample(2.0, 3, 3, 3, 250, 1, 0.0)
    assert per_sample_cores([a, b, c], 100) == [1.0, 1.5]
