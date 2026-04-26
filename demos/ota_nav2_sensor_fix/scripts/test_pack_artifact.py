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


def test_merge_catalog_creates_file(tmp_path):
    catalog = tmp_path / "catalog.json"
    entry = {"id": "a", "name": "a"}
    pack_artifact.merge_catalog(catalog, entry)
    data = json.loads(catalog.read_text())
    assert data == [entry]


def test_merge_catalog_appends(tmp_path):
    catalog = tmp_path / "catalog.json"
    catalog.write_text(json.dumps([{"id": "a", "name": "a"}]))
    entry = {"id": "b", "name": "b"}
    pack_artifact.merge_catalog(catalog, entry)
    data = json.loads(catalog.read_text())
    assert [e["id"] for e in data] == ["a", "b"]


def test_merge_catalog_replaces_same_id(tmp_path):
    catalog = tmp_path / "catalog.json"
    catalog.write_text(json.dumps([{"id": "a", "name": "old"}]))
    entry = {"id": "a", "name": "new"}
    pack_artifact.merge_catalog(catalog, entry)
    data = json.loads(catalog.read_text())
    assert data == [entry]


def test_create_tarball(tmp_path):
    install = tmp_path / "install" / "fixed_lidar"
    (install / "lib").mkdir(parents=True)
    (install / "lib" / "fixed_lidar_node").write_text("binary")
    out_dir = tmp_path / "artifacts"
    out_path = pack_artifact.create_tarball(
        package="fixed_lidar",
        version="2.1.0",
        install_dir=install,
        out_dir=out_dir,
    )
    assert out_path == out_dir / "fixed_lidar-2.1.0.tar.gz"
    assert out_path.exists()
    with tarfile.open(out_path) as tf:
        names = tf.getnames()
    assert "fixed_lidar/lib/fixed_lidar_node" in names
