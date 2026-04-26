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
        default="0.0.0",
        help="Semantic version of the artifact (pass '' for uninstall).",
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


def slug(package: str, version: str) -> str:
    return f"{package}_{version.replace('.', '_')}" if version else package


def build_entry(
    *,
    package: str,
    version: str,
    kind: Kind,
    target_component: str,
    executable: str,
    notes: str,
    duration: int,
    size_bytes: int,
) -> dict:
    entry: dict = {
        "id": slug(package, version) if kind != "uninstall" else f"{package}_remove",
        "name": f"{package} {version}".strip(),
        "automated": False,
        "origins": ["remote"],
        "notes": notes,
        "duration": duration,
    }
    if version:
        entry["version"] = version
    if size_bytes > 0:
        entry["size"] = max(1, size_bytes // 1024)

    if kind == "update":
        entry["updated_components"] = [target_component]
    elif kind == "install":
        entry["added_components"] = [target_component]
    else:  # uninstall
        entry["removed_components"] = [target_component]

    if kind != "uninstall":
        entry["x_medkit_artifact_url"] = f"/artifacts/{package}-{version}.tar.gz"
        entry["x_medkit_target_package"] = package
        if executable:
            entry["x_medkit_executable"] = executable
    else:
        entry["x_medkit_target_package"] = package

    return entry


def merge_catalog(catalog_path: Path, entry: dict) -> None:
    catalog_path = Path(catalog_path)
    catalog_path.parent.mkdir(parents=True, exist_ok=True)
    if catalog_path.exists():
        data = json.loads(catalog_path.read_text())
    else:
        data = []
    data = [e for e in data if e.get("id") != entry["id"]]
    data.append(entry)
    catalog_path.write_text(json.dumps(data, indent=2) + "\n")


def create_tarball(
    *,
    package: str,
    version: str,
    install_dir: Path,
    out_dir: Path,
) -> Path:
    install_dir = Path(install_dir)
    if not install_dir.exists():
        raise FileNotFoundError(f"install dir does not exist: {install_dir}")
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / f"{package}-{version}.tar.gz"
    with tarfile.open(out_path, "w:gz") as tf:
        tf.add(install_dir, arcname=package)
    return out_path


def colcon_build(workspace: Path, package: str) -> None:
    cmd = ["colcon", "build", "--packages-select", package, "--symlink-install"]
    completed = subprocess.run(cmd, cwd=workspace, check=False)
    if completed.returncode != 0:
        raise SystemExit(f"colcon build failed for {package}")


def run(
    *,
    package: str,
    version: str,
    kind: Kind,
    target_component: str,
    executable: str,
    notes: str,
    duration: int,
    out_dir: str,
    catalog: str,
    skip_build: bool,
    workspace: str,
) -> int:
    if kind == "install" and not executable:
        sys.stderr.write("--executable is required for install\n")
        raise SystemExit(2)
    if kind != "uninstall" and not version:
        sys.stderr.write(f"--version is required for kind={kind}\n")
        raise SystemExit(2)

    out_dir_p = Path(out_dir)
    catalog_p = Path(catalog)
    workspace_p = Path(workspace)

    size_bytes = 0
    if kind != "uninstall":
        if not skip_build:
            colcon_build(workspace_p, package)
        install_dir = workspace_p / "install" / package
        if not install_dir.exists():
            sys.stderr.write(f"install dir missing: {install_dir}\n")
            raise SystemExit(3)
        tarball = create_tarball(
            package=package,
            version=version,
            install_dir=install_dir,
            out_dir=out_dir_p,
        )
        size_bytes = tarball.stat().st_size

    entry = build_entry(
        package=package,
        version=version,
        kind=kind,
        target_component=target_component,
        executable=executable,
        notes=notes,
        duration=duration,
        size_bytes=size_bytes,
    )
    merge_catalog(catalog_p, entry)
    print(f"packed {entry['id']}")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return run(**vars(args))


if __name__ == "__main__":
    sys.exit(main())
