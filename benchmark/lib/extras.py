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


_EXTRA_PID_FILE = "/tmp/extra_cmd.pid"


def check_extras(container, plugin_paths, has_extra_cmd, expected_entities, entity_check,
                 run=None, end_cmd=None):
    """Did the extras survive the window: plugins loaded, the extra process still alive.

    ``entity_check`` returns the entity probe dict (or None). A plugin the gateway
    logged as failed to load, a dead extra process or a missing entity are
    ``problems``; the lane is partial when there is one. ``end_cmd`` runs once in the
    container after the window; its output goes to ``end_cmd_output``.
    """
    from benchmark.lib import docker_helpers as dh
    run = run or dh.run
    problems = []
    out = {"problems": problems}
    if expected_entities:
        ent = entity_check()
        out.update(ent)
        if ent["missing"]:
            problems.append(f"entities not listed: {', '.join(ent['missing'])}")
    if plugin_paths:
        logs = run(["docker", "logs", container], merge_stderr=True)
        failed = [p for p in plugin_paths if f"Failed to load plugin from {p}" in logs]
        out["plugin_load_failures"] = failed
        problems += [f"plugin failed to load: {p}" for p in failed]
    if has_extra_cmd:
        try:
            # An exited child stays a zombie until the container's init reaps it.
            run(["docker", "exec", container, "sh", "-c",
                 f"p=$(cat {_EXTRA_PID_FILE}); [ -r /proc/$p/status ] && "
                 f"! grep -q '^State:.*Z' /proc/$p/status"])
            out["extra_process_alive"] = True
        except RuntimeError:
            out["extra_process_alive"] = False
            problems.append("the extra process exited during the run")
    if end_cmd:
        try:
            out["end_cmd_output"] = dh.exec_via_file(container, end_cmd, timeout=60).strip()
        except RuntimeError as exc:
            out["end_cmd_output"] = None
            problems.append(f"the end command failed: {exc}")
    return out
