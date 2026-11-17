"""Extra gateway plugins and one extra process for the footprint lane."""
from __future__ import annotations
import json

import yaml


_ENTITY_ENDPOINTS = ("/apps", "/components", "/areas", "/functions")


def _kv(text, what):
    if "=" not in text:
        raise ValueError(f"{what} must be KEY=VALUE: {text!r}")
    key, value = text.split("=", 1)
    return key, yaml.safe_load(value)


def build_extra_env(plugins, plugin_params, gateway_params, extra_cmd):
    """Environment for the benchmark container from the footprint CLI options.

    ``plugins``        ["NAME=/path/libname.so", ...]
    ``plugin_params``  ["NAME.KEY=VALUE", ...]  (VALUE is read as YAML)
    ``gateway_params`` ["KEY=VALUE", ...]       (VALUE is read as YAML)
    ``extra_cmd``      one command line started in the container, or None
    """
    specs = {}
    for text in plugins or []:
        if "=" not in text:
            raise ValueError(f"--plugin must be NAME=PATH: {text!r}")
        name, path = text.split("=", 1)
        specs[name] = {"name": name, "path": path, "config": {}}
    for text in plugin_params or []:
        dotted, value = _kv(text, "--plugin-param")
        name, _, key = dotted.partition(".")
        if name not in specs or not key:
            raise ValueError(f"--plugin-param {text!r} names no --plugin NAME.KEY")
        specs[name]["config"][key] = value
    env = {}
    if specs:
        env["BENCH_PLUGINS_JSON"] = json.dumps(list(specs.values()))
    if gateway_params:
        env["BENCH_GATEWAY_PARAMS_JSON"] = json.dumps(
            dict(_kv(t, "--gateway-param") for t in gateway_params))
    if extra_cmd:
        env["BENCH_EXTRA_CMD"] = extra_cmd
    return env


def entity_ids(container, port, api_base, curl):
    """IDs of all entities the gateway lists, over the entity collections."""
    ids = set()
    for ep in _ENTITY_ENDPOINTS:
        try:
            code, body = curl(container, f"http://localhost:{port}{api_base}{ep}")
        except Exception:
            continue
        if code != 200:
            continue
        try:
            items = json.loads(body).get("items", [])
        except ValueError:
            continue
        ids.update(i["id"] for i in items if isinstance(i, dict) and "id" in i)
    return ids


def check_entities(container, port, api_base, curl, expected):
    """Which of the expected entity IDs the gateway lists."""
    found = entity_ids(container, port, api_base, curl)
    return {"expected": sorted(expected),
            "found": sorted(set(expected) & found),
            "missing": sorted(set(expected) - found),
            "listed_total": len(found)}
