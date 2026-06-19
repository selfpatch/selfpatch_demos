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
from benchmark.benchmark import load_override_sets

CFG = Path(__file__).resolve().parents[1] / "configs" / "overrides.yaml"


def test_override_sets():
    sets = load_override_sets(str(CFG))
    assert sets["refresh_fast"]["refresh_interval_ms"] == 200
    assert sets["sse_disabled"]["sse"]["max_subscriptions"] == 0
