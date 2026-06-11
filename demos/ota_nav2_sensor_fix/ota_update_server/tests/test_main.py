"""Tests for the FastAPI update server."""
from __future__ import annotations

import json
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from ota_update_server import create_app


@pytest.fixture
def artifacts_dir(tmp_path) -> Path:
    return tmp_path


@pytest.fixture
def client(artifacts_dir):
    return TestClient(create_app(artifacts_dir))


def test_catalog_empty_when_missing(client):
    resp = client.get("/catalog")
    assert resp.status_code == 200
    assert resp.json() == []


def test_catalog_returns_file_contents(client, artifacts_dir):
    payload = [
        {"id": "fixed_lidar_2_1_0", "updated_components": ["scan_sensor_node"]},
        {"id": "obstacle_classifier_v2_install", "added_components": ["obstacle_classifier"]},
    ]
    (artifacts_dir / "catalog.json").write_text(json.dumps(payload))
    resp = client.get("/catalog")
    assert resp.status_code == 200
    assert resp.json() == payload


def test_artifact_returns_file(client, artifacts_dir):
    (artifacts_dir / "fixed_lidar-2.1.0.tar.gz").write_bytes(b"BIN")
    resp = client.get("/artifacts/fixed_lidar-2.1.0.tar.gz")
    assert resp.status_code == 200
    assert resp.content == b"BIN"


def test_artifact_404_when_missing(client):
    resp = client.get("/artifacts/missing.tar.gz")
    assert resp.status_code == 404


def test_artifact_rejects_path_traversal(client, artifacts_dir):
    (artifacts_dir.parent / "secret.txt").write_text("hush")
    resp = client.get("/artifacts/..%2Fsecret.txt")
    assert resp.status_code in (400, 404)
