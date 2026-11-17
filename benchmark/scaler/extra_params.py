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
