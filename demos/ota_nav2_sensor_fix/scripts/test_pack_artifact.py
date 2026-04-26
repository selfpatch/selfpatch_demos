"""Tests for pack_artifact.py."""
from __future__ import annotations

import json
import tarfile
from pathlib import Path

import pytest

import pack_artifact


def test_imports():
    assert hasattr(pack_artifact, "main")


def test_main_requires_package():
    with pytest.raises(SystemExit):
        pack_artifact.main([])


def test_main_parses_basic_args(monkeypatch, tmp_path):
    captured = {}

    def fake_run(**kwargs):
        captured.update(kwargs)
        return 0

    monkeypatch.setattr(pack_artifact, "run", fake_run)
    rc = pack_artifact.main(
        [
            "--package", "fixed_lidar",
            "--version", "2.1.0",
            "--kind", "update",
            "--target-component", "scan_sensor_node",
            "--executable", "fixed_lidar_node",
            "--notes", "noise filter fix",
            "--out-dir", str(tmp_path / "artifacts"),
            "--catalog", str(tmp_path / "artifacts" / "catalog.json"),
            "--skip-build",
        ]
    )
    assert rc == 0
    assert captured["package"] == "fixed_lidar"
    assert captured["version"] == "2.1.0"
    assert captured["kind"] == "update"
    assert captured["target_component"] == "scan_sensor_node"
    assert captured["executable"] == "fixed_lidar_node"
    assert captured["notes"] == "noise filter fix"
    assert captured["skip_build"] is True
