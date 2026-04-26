#!/usr/bin/env python3
"""Pack a ROS 2 package into an OTA artifact + SOVD-shaped catalog entry."""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tarfile
from pathlib import Path
from typing import Literal

Kind = Literal["update", "install", "uninstall"]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Pack a ROS 2 package into an OTA artifact + SOVD catalog entry.",
    )
    parser.add_argument("--package", required=True, help="ROS 2 package name to pack.")
    parser.add_argument(
        "--version",
        default="",
        help="Semantic version of the artifact (omit for uninstall).",
    )
    parser.add_argument(
        "--kind",
        required=True,
        choices=["update", "install", "uninstall"],
        help="Catalog entry kind.",
    )
    parser.add_argument(
        "--target-component",
        required=True,
        help="SOVD component the entry targets.",
    )
    parser.add_argument(
        "--executable",
        default="",
        help="Executable name inside install/<package>/lib (required for install).",
    )
    parser.add_argument("--notes", default="", help="Free-text notes for the catalog entry.")
    parser.add_argument(
        "--duration",
        type=int,
        default=10,
        help="Estimated install duration in seconds.",
    )
    parser.add_argument(
        "--out-dir",
        default="artifacts",
        help="Output directory for tarballs.",
    )
    parser.add_argument(
        "--catalog",
        default="artifacts/catalog.json",
        help="Path to the SOVD catalog JSON file.",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Skip running colcon build; reuse existing install/ tree.",
    )
    parser.add_argument(
        "--workspace",
        default=".",
        help="Path to the colcon workspace root.",
    )
    return parser


def run(**kwargs) -> int:
    raise NotImplementedError


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return run(**vars(args))


if __name__ == "__main__":
    sys.exit(main())
