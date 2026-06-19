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
import argparse
import json
import os
from pathlib import Path
from unittest.mock import patch

import pytest

from benchmark.benchmark import (
    _args_dict,
    _read_demo_refresh_ms,
    _write_metadata,
    check_host_load,
)

FIXTURE_PARAMS = (
    Path(__file__).resolve().parent / "fixtures" / "medkit_params.yaml"
)


def test_args_dict_strips_func():
    """C5: vars(args) with func entry removed."""
    ns = argparse.Namespace(cmd="footprint", duration=300, func=lambda: None)
    d = _args_dict(ns)
    assert "func" not in d
    assert d["cmd"] == "footprint"
    assert d["duration"] == 300


def test_read_demo_refresh_ms_fixture():
    """C3: reads refresh_interval_ms from the fixture params file."""
    val = _read_demo_refresh_ms(FIXTURE_PARAMS)
    assert val == 1000


def test_read_demo_refresh_ms_missing_file():
    """C3: returns 'unknown' when the file does not exist."""
    val = _read_demo_refresh_ms("/nonexistent/path/medkit_params.yaml")
    assert val == "unknown"


def test_read_demo_refresh_ms_real_demo():
    """C3: reads the real turtlebot3 params and gets an integer."""
    repo = Path(__file__).resolve().parents[3]
    params = repo / "demos" / "turtlebot3_integration" / "config" / "medkit_params.yaml"
    if params.exists():
        val = _read_demo_refresh_ms(params)
        assert isinstance(val, int), f"expected int, got {val!r}"
        assert val > 0


def test_check_host_load_normal(monkeypatch):
    """C4: no warning when load is below cpu_count."""
    monkeypatch.setattr(os, "getloadavg", lambda: (0.1, 0.1, 0.1))
    monkeypatch.setattr(os, "cpu_count", lambda: 8)
    result = check_host_load(strict=False)
    assert result["high_host_load"] is False
    assert result["host_load1"] == pytest.approx(0.1)


def test_check_host_load_high_warns(monkeypatch, capsys):
    """C4: prints a warning when load exceeds cpu_count."""
    monkeypatch.setattr(os, "getloadavg", lambda: (16.0, 10.0, 8.0))
    monkeypatch.setattr(os, "cpu_count", lambda: 8)
    result = check_host_load(strict=False)
    assert result["high_host_load"] is True
    out = capsys.readouterr().out
    assert "WARNING" in out


def test_check_host_load_strict_raises(monkeypatch):
    """C4: raises RuntimeError in strict mode when load is high."""
    monkeypatch.setattr(os, "getloadavg", lambda: (16.0, 10.0, 8.0))
    monkeypatch.setattr(os, "cpu_count", lambda: 8)
    with pytest.raises(RuntimeError, match="--strict"):
        check_host_load(strict=True)


def test_write_metadata_fields(tmp_path, monkeypatch):
    """C3: _write_metadata writes run_metadata.json with expected keys."""
    monkeypatch.setattr(os, "getloadavg", lambda: (0.5, 0.5, 0.5))
    monkeypatch.setattr(os, "cpu_count", lambda: 4)
    args = argparse.Namespace(cmd="footprint", demo="turtlebot3", duration=60, repeats=1)
    cell_meta = {"gateway_sha": "deadbeef", "image_digest": "sha256:cafe", "allocator": "glibc"}
    _write_metadata(tmp_path, args, cell_meta, refresh_ms=10000)
    data = json.loads((tmp_path / "run_metadata.json").read_text())
    assert data["gateway_sha"] == "deadbeef"
    assert data["image_digest"] == "sha256:cafe"
    assert data["allocator"] == "glibc"
    assert data["refresh_interval_ms"] == 10000
    assert "func" not in data.get("args", {})
    assert "high_host_load" in data
    assert "host_load1" in data


def test_write_metadata_no_func_key(tmp_path, monkeypatch):
    """C5: func key is never present in run_metadata.json args."""
    monkeypatch.setattr(os, "getloadavg", lambda: (0.1, 0.1, 0.1))
    monkeypatch.setattr(os, "cpu_count", lambda: 4)
    args = argparse.Namespace(cmd="scaling", func=lambda: None, entities="10,50")
    _write_metadata(tmp_path, args, {}, refresh_ms=1000)
    data = json.loads((tmp_path / "run_metadata.json").read_text())
    assert "func" not in data["args"]


def test_strip_drops_load_stats():
    from benchmark.benchmark import _strip
    out = _strip({"uss_kib": 1.0, "load_stats": {"p50_ms": 5}, "_meta": {}, "status": "ok"})
    assert "load_stats" not in out and "_meta" not in out and out["uss_kib"] == 1.0
