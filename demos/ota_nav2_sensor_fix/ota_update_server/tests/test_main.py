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
