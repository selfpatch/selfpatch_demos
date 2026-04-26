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


def main(argv: list[str] | None = None) -> int:
    raise NotImplementedError


if __name__ == "__main__":
    sys.exit(main())
