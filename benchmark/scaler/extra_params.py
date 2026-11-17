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
"""Write a gateway params file that loads extra plugins, from environment JSON.

BENCH_PLUGINS_JSON         [{"name": "p", "path": "/x/libp.so", "config": {"k": 1}}]
BENCH_GATEWAY_PARAMS_JSON  {"discovery.mode": "hybrid"}

The file targets the gateway node the profiles start (/diagnostics/ros2_medkit_gateway)
and goes to stdout. Plugin settings are flat dotted keys, as the gateway reads them.
"""
from __future__ import annotations
import json
import os
import sys

import yaml

GATEWAY_NODE = "/diagnostics/ros2_medkit_gateway"


def build_params(plugins, gateway_params):
    params = dict(gateway_params)
    if plugins:
        params["plugins"] = [p["name"] for p in plugins]
        for p in plugins:
            params[f"plugins.{p['name']}.path"] = p["path"]
            for key, value in (p.get("config") or {}).items():
                params[f"plugins.{p['name']}.{key}"] = value
    return {GATEWAY_NODE: {"ros__parameters": params}}


def main():
    plugins = json.loads(os.environ.get("BENCH_PLUGINS_JSON") or "[]")
    gateway_params = json.loads(os.environ.get("BENCH_GATEWAY_PARAMS_JSON") or "{}")
    yaml.safe_dump(build_params(plugins, gateway_params), sys.stdout)


if __name__ == "__main__":
    sys.exit(main())
