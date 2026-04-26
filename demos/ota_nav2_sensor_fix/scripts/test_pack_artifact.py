"""Tests for pack_artifact.py."""
from __future__ import annotations

import json
import tarfile
from pathlib import Path

import pytest

import pack_artifact


def test_imports():
    assert hasattr(pack_artifact, "main")
