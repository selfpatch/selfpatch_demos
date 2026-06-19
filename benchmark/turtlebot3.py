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
"""turtlebot3_integration demo wiring (a dataclass; no ABC for one demo)."""
from __future__ import annotations
from dataclasses import dataclass, field
from benchmark.lib.config_sweep import OVERRIDE_ROOT


@dataclass
class DemoConfig:
    name: str
    compose_file: str
    compose_profile: str
    gateway_service: str
    gateway_proc: str
    domain_id: int
    api_base: str
    base_params_rel: str
    override_root: list = field(default_factory=lambda: list(OVERRIDE_ROOT))


TURTLEBOT3 = DemoConfig(
    name="turtlebot3",
    compose_file="demos/turtlebot3_integration/docker-compose.yml",
    compose_profile="ci",  # headless Gazebo (no DISPLAY) - correct for benchmarking
    gateway_service="turtlebot3-demo-ci",
    gateway_proc="gateway_node",
    domain_id=30,
    api_base="/api/v1",
    base_params_rel="demos/turtlebot3_integration/config/medkit_params.yaml",
)
