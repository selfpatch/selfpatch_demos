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
from benchmark.scaler.spawn_nodes import plan_graph


def test_counts_and_type_cardinality():
    specs = plan_graph(4, 2, 1, 2, 3)
    assert len(specs) == 4
    assert all(len(s["topics"]) == 2 for s in specs)
    assert len({t["type"] for s in specs for t in s["topics"]}) == 2
    assert all(len(s["params"]) == 3 for s in specs)


def test_unique_names():
    assert len({s["name"] for s in plan_graph(3, 1, 0, 1, 0)}) == 3
