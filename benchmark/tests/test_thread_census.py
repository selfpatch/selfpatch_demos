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
"""Unit tests for the thread census categorization logic in docker_helpers."""
from benchmark.lib.docker_helpers import _categorize_comm


def test_rclcpp_prefix():
    assert _categorize_comm("rclcpp") == "rclcpp_executor"
    assert _categorize_comm("rclcpp_cb") == "rclcpp_executor"
    assert _categorize_comm("RCLCPP_EXEC") == "rclcpp_executor"


def test_httplib_prefix():
    assert _categorize_comm("httplib") == "httplib"
    assert _categorize_comm("httplib_worker") == "httplib"


def test_fastdds_prefix():
    assert _categorize_comm("fastdds") == "dds"
    assert _categorize_comm("FastDDS_RECV") == "dds"


def test_dds_prefix():
    assert _categorize_comm("dds_recv") == "dds"
    assert _categorize_comm("DDS_SEND") == "dds"


def test_dq_prefix():
    assert _categorize_comm("dq_io") == "dds"


def test_cdds_prefix():
    assert _categorize_comm("cdds_recv") == "dds"
    assert _categorize_comm("CDDS_IO") == "dds"


def test_rcl_infra():
    assert _categorize_comm("rcl_context") == "rclcpp_executor"
    assert _categorize_comm("intra_proc") == "rclcpp_executor"
    assert _categorize_comm("waitset_0") == "rclcpp_executor"


def test_plugin_prefix():
    assert _categorize_comm("plugin_loader") == "plugin"
    assert _categorize_comm("Plugin") == "plugin"


def test_gateway_comm():
    assert _categorize_comm("gateway_node") == "gateway"


def test_other():
    assert _categorize_comm("sh") == "other"
    assert _categorize_comm("") == "other"
    assert _categorize_comm("unknown_thread") == "other"


def _run_census_from_comm_list(comms):
    """Simulate what thread_census does from a list of comm strings."""
    counts: dict = {}
    total = 0
    for comm in comms:
        comm = comm.strip()
        if not comm:
            continue
        cat = _categorize_comm(comm)
        counts[cat] = counts.get(cat, 0) + 1
        total += 1
    counts["total"] = total
    return counts


def test_census_groups_correctly():
    comms = [
        "rclcpp_exec", "rclcpp_exec",
        "httplib_w", "httplib_w", "httplib_w",
        "fastdds_recv",
        "plugin_x",
        "gateway_node",
    ]
    result = _run_census_from_comm_list(comms)
    assert result["rclcpp_executor"] == 2
    assert result["httplib"] == 3
    assert result["dds"] == 1
    assert result["plugin"] == 1
    assert result["gateway"] == 1
    assert result.get("other", 0) == 0
    assert result["total"] == 8


def test_census_empty():
    result = _run_census_from_comm_list([])
    assert result == {"total": 0}


def test_census_skips_blank_lines():
    result = _run_census_from_comm_list(["", "  ", "rclcpp_cb"])
    assert result["rclcpp_executor"] == 1
    assert result["total"] == 1
