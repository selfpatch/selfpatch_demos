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
"""Capture run conditions so two runs are comparable."""
from __future__ import annotations
import datetime
import os
import platform


def _local(path):
    try:
        with open(path) as fh:
            return fh.read()
    except Exception:
        return ""


def collect(args, demo_name, cell_meta=None, refresh_ms=None):
    """Build a run metadata dict from host-local sources and per-cell metadata.

    ``cell_meta`` is the ``_meta`` dict captured by ``runner.run_cell`` while
    the container was still alive.  It carries ``gateway_sha``, ``image_digest``,
    and ``allocator``.  When ``cell_meta`` is None (no container ran), these
    fields fall back to "unknown".

    ``args`` must already have the ``func`` entry removed by the caller
    (pass ``{k: v for k, v in vars(parsed_args).items() if k != 'func'}``).
    """
    if cell_meta is None:
        cell_meta = {}

    # Host CPU: read /proc/cpuinfo; fall back to platform.processor() in
    # environments where the file exists but has no "model name" line
    # (e.g. devcontainers / ARM hosts).
    cpu_model = "unknown"
    for line in _local("/proc/cpuinfo").splitlines():
        if line.startswith("model name"):
            cpu_model = line.split(":", 1)[1].strip()
            break
    if cpu_model == "unknown":
        fallback = platform.processor() or platform.machine()
        if fallback:
            cpu_model = fallback

    mem_total_kb = 0
    for line in _local("/proc/meminfo").splitlines():
        if line.startswith("MemTotal"):
            try:
                mem_total_kb = int(line.split()[1])
            except (IndexError, ValueError):
                pass
            break

    return {
        "args": args,
        "demo": demo_name,
        "image_digest": cell_meta.get("image_digest", "unknown"),
        "gateway_sha": cell_meta.get("gateway_sha", "unknown"),
        "nproc": os.cpu_count() or 0,
        "cpu_model": cpu_model,
        "mem_total_kb": mem_total_kb,
        "kernel": platform.release(),
        "ros_distro": os.environ.get("ROS_DISTRO", "jazzy"),
        "dds_impl": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        "allocator": cell_meta.get("allocator", "unknown"),
        "refresh_interval_ms": refresh_ms,
        "host_load1": os.getloadavg()[0],
        "timestamp_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }
