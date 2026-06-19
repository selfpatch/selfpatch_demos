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
import platform

from benchmark.lib.runmeta import collect


def test_required_fields():
    meta = collect({"cmd": "footprint"}, "turtlebot3",
                   cell_meta=None, refresh_ms=1000)
    for k in ("args", "demo", "image_digest", "nproc", "cpu_model", "mem_total_kb",
              "kernel", "ros_distro", "dds_impl", "allocator", "refresh_interval_ms",
              "host_load1", "timestamp_utc"):
        assert k in meta


def test_cell_meta_propagated():
    """C2: gateway_sha, image_digest, allocator come from cell_meta, not a live container."""
    cell = {"gateway_sha": "abc123", "image_digest": "sha256:dead", "allocator": "jemalloc"}
    meta = collect({"cmd": "footprint"}, "turtlebot3",
                   cell_meta=cell, refresh_ms=500)
    assert meta["gateway_sha"] == "abc123"
    assert meta["image_digest"] == "sha256:dead"
    assert meta["allocator"] == "jemalloc"
    assert meta["refresh_interval_ms"] == 500


def test_cell_meta_none_defaults_unknown():
    """When no cell_meta provided, fields default to 'unknown' gracefully."""
    meta = collect({"cmd": "footprint"}, "demo", cell_meta=None, refresh_ms=None)
    assert meta["gateway_sha"] == "unknown"
    assert meta["image_digest"] == "unknown"
    assert meta["allocator"] == "unknown"


def test_func_key_not_in_args():
    """C5: the caller must strip 'func' before passing args; collect does not crash on extras."""
    args = {"cmd": "footprint", "duration": 300}
    meta = collect(args, "demo", cell_meta={}, refresh_ms=1000)
    # collect passes args through unchanged - caller is responsible for stripping func
    assert meta["args"] == args


def test_cpu_model_fallback():
    """C5: cpu_model never stays 'unknown' on a machine with platform.processor()."""
    meta = collect({}, "demo", cell_meta={}, refresh_ms=None)
    # On any reasonable host, cpu_model should be non-empty.
    # If /proc/cpuinfo lacks 'model name', platform.processor() or .machine() kicks in.
    assert meta["cpu_model"] != ""
    # The field must be present and a string.
    assert isinstance(meta["cpu_model"], str)
    # If platform gives us something, it should match (or /proc/cpuinfo gave us something).
    fallback = platform.processor() or platform.machine()
    if fallback:
        # Either source gave us a non-"unknown" value.
        assert meta["cpu_model"] != "unknown" or not fallback
