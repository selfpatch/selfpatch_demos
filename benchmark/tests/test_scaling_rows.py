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
from benchmark.benchmark import scaling_rows


def test_uss_per_entity():
    cells = [{"entity_count": 10, "uss_kib_median": 50000},
             {"entity_count": 100, "uss_kib_median": 600000}]
    rows = scaling_rows(cells)
    assert rows[0]["uss_per_entity"] == 5000 and rows[1]["uss_per_entity"] == 6000
