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


def test_build_entry_update_kind():
    entry = pack_artifact.build_entry(
        package="fixed_lidar",
        version="2.1.0",
        kind="update",
        target_component="scan_sensor_node",
        executable="fixed_lidar_node",
        notes="fix noise",
        duration=10,
        size_bytes=2048,
    )
    assert entry["id"] == "fixed_lidar_2_1_0"
    assert entry["name"] == "fixed_lidar 2.1.0"
    assert entry["version"] == "2.1.0"
    assert entry["automated"] is False
    assert entry["origins"] == ["remote"]
    assert entry["notes"] == "fix noise"
    assert entry["size"] == 2  # KB rounded
    assert entry["duration"] == 10
    assert entry["updated_components"] == ["scan_sensor_node"]
    assert "added_components" not in entry
    assert "removed_components" not in entry
    assert entry["x_medkit_target_package"] == "fixed_lidar"
    assert entry["x_medkit_executable"] == "fixed_lidar_node"
    assert entry["x_medkit_artifact_url"] == "/artifacts/fixed_lidar-2.1.0.tar.gz"


def test_build_entry_install_kind():
    entry = pack_artifact.build_entry(
        package="obstacle_classifier_v2",
        version="1.0.0",
        kind="install",
        target_component="obstacle_classifier",
        executable="obstacle_classifier_node",
        notes="extra safety",
        duration=15,
        size_bytes=4096,
    )
    assert entry["added_components"] == ["obstacle_classifier"]
    assert "updated_components" not in entry
    assert "removed_components" not in entry


def test_build_entry_uninstall_kind():
    entry = pack_artifact.build_entry(
        package="broken_lidar_legacy",
        version="",
        kind="uninstall",
        target_component="broken_lidar_legacy",
        executable="",
        notes="cleanup",
        duration=5,
        size_bytes=0,
    )
    assert entry["removed_components"] == ["broken_lidar_legacy"]
    assert "added_components" not in entry
    assert "updated_components" not in entry
    assert "x_medkit_artifact_url" not in entry
    assert "x_medkit_executable" not in entry
