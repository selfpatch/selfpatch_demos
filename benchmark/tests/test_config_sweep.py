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
import yaml
from benchmark.lib.config_sweep import deep_merge, apply_override_at, OVERRIDE_ROOT

FIX = Path(__file__).parent / "fixtures"


def _p():
    return yaml.safe_load((FIX / "medkit_params.yaml").read_text())


def test_deep_merge_keeps_siblings():
    base = {"a": {"b": 1, "c": 2}}
    assert deep_merge(base, {"a": {"b": 9}}) == {"a": {"b": 9, "c": 2}}
    assert base["a"]["b"] == 1


def test_apply_override_keeps_fault_manager():
    out = apply_override_at(_p(), OVERRIDE_ROOT, {"refresh_interval_ms": 200})
    root = out["diagnostics"]["ros2_medkit_gateway"]["ros__parameters"]
    assert root["refresh_interval_ms"] == 200
    assert root["discovery"]["mode"] == "hybrid"
    assert out["diagnostics"]["fault_manager"]["ros__parameters"]["confirmation_threshold"] == 3


def test_apply_override_disables_sse():
    out = apply_override_at(_p(), OVERRIDE_ROOT, {"sse": {"max_subscriptions": 0}})
    root = out["diagnostics"]["ros2_medkit_gateway"]["ros__parameters"]
    assert root["sse"]["max_subscriptions"] == 0 and root["sse"]["max_clients"] == 10
