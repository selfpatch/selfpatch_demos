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
"""Derive gateway param variants by merging overrides at the namespaced root."""
from __future__ import annotations
import copy

OVERRIDE_ROOT = ["diagnostics", "ros2_medkit_gateway", "ros__parameters"]


def deep_merge(base, override):
    out = copy.deepcopy(base)
    for k, v in override.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = copy.deepcopy(v)
    return out


def apply_override_at(params, root_path, override):
    out = copy.deepcopy(params)
    node = out
    for key in root_path[:-1]:
        node = node[key]
    node[root_path[-1]] = deep_merge(node[root_path[-1]], override)
    return out
