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

import pytest

import benchmark.benchmark as bm
from benchmark.benchmark import (
    RESULTS,
    _args_dict,
    _latest_run_dir,
    _read_demo_refresh_ms,
    _resolve_root,
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


# ---------------------------------------------------------------------------
# _resolve_root: --run-dir threading so lanes can share one run dir
# ---------------------------------------------------------------------------

def test_resolve_root_explicit_root_wins(tmp_path):
    """An explicit root (cmd_all) overrides --run-dir."""
    ns = argparse.Namespace(run_dir="ignored")
    assert _resolve_root(ns, tmp_path) == tmp_path


def test_resolve_root_uses_run_dir():
    ns = argparse.Namespace(run_dir="myrun")
    assert _resolve_root(ns, None) == RESULTS / "myrun"


def test_resolve_root_none_when_unset():
    ns = argparse.Namespace(run_dir=None)
    assert _resolve_root(ns, None) is None


# ---------------------------------------------------------------------------
# _latest_run_dir: clear errors instead of FileNotFoundError/IndexError
# ---------------------------------------------------------------------------

def test_latest_run_dir_missing_dir(monkeypatch, tmp_path):
    monkeypatch.setattr(bm, "RESULTS", tmp_path / "does_not_exist")
    with pytest.raises(RuntimeError, match="no results"):
        _latest_run_dir()


def test_latest_run_dir_empty_dir(monkeypatch, tmp_path):
    monkeypatch.setattr(bm, "RESULTS", tmp_path)
    with pytest.raises(RuntimeError, match="no results found"):
        _latest_run_dir()


def test_latest_run_dir_returns_most_recent(monkeypatch, tmp_path):
    monkeypatch.setattr(bm, "RESULTS", tmp_path)
    (tmp_path / "20260101-000000").mkdir()
    (tmp_path / "20260202-000000").mkdir()
    assert _latest_run_dir().name == "20260202-000000"


# ---------------------------------------------------------------------------
# cmd_compare: high_host_load must gate on ANY lane, not just the first
# ---------------------------------------------------------------------------

def _write_lane_meta(run_dir, lane, meta):
    d = run_dir / lane
    d.mkdir(parents=True)
    (d / "run_metadata.json").write_text(json.dumps(meta))


def test_cmd_compare_high_load_on_any_lane_gates(tmp_path):
    """A noisy run on ANY lane (not only the first one read) must abort compare
    with exit 2 - the first lane is clean, the second is high-load."""
    run_dir = tmp_path / "run1"
    host = {"cpu_model": "X", "nproc": 4}
    # Alphabetically 'footprint' sorts before 'scaling'; put the high-load flag on
    # the LATER lane to prove the gate does not stop at the first lane's metadata.
    _write_lane_meta(run_dir, "footprint", {**host, "high_host_load": False})
    _write_lane_meta(run_dir, "scaling", {**host, "high_host_load": True})
    baseline = tmp_path / "bl.json"
    baseline.write_text(json.dumps({"host": host}))

    args = argparse.Namespace(run=str(run_dir), baseline=str(baseline))
    with pytest.raises(SystemExit) as ei:
        bm.cmd_compare(args)
    assert ei.value.code == 2


def test_cmd_compare_clean_lanes_pass_host_check(tmp_path, capsys):
    """All lanes clean + host matches + no comparable metrics -> no SystemExit
    (compare prints 'No comparable lanes' and returns)."""
    run_dir = tmp_path / "run2"
    host = {"cpu_model": "X", "nproc": 4}
    _write_lane_meta(run_dir, "footprint", {**host, "high_host_load": False})
    baseline = tmp_path / "bl.json"
    baseline.write_text(json.dumps({"host": host}))

    args = argparse.Namespace(run=str(run_dir), baseline=str(baseline))
    bm.cmd_compare(args)  # must not raise (no summary.json -> no metrics)
    assert "No comparable lanes" in capsys.readouterr().out
