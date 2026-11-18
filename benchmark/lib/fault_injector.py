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
"""Fault injection helpers for the fault lane benchmark.

Pure functions are separated from the docker layer so they can be unit-tested
without a running container.
"""
from __future__ import annotations

from typing import List

from benchmark.lib import docker_helpers as dh


# ReportFault.srv request fields (verified against ros2_medkit_msgs source):
#   string source_id
#   string fault_code
#   uint8 event_type   (EVENT_FAILED=0, EVENT_PASSED=1)
#   uint8 severity     (INFO=0, WARNING=1, ERROR=2, CRITICAL=3)
#   string description
_SEVERITY_CRITICAL = 3
_EVENT_FAILED = 0
# The node scaler/topic_source.py runs as; its published topic is what a fault captures.
SOURCE_ID = "/bench_source"
_SERVICE = "/fault_manager/report_fault"
_SERVICE_TYPE = "ros2_medkit_msgs/srv/ReportFault"


def build_inject_yaml(source_id: str, fault_code: str, description: str) -> str:
    """Return the YAML string for one ros2 service call request.

    Uses CRITICAL severity so the fault bypasses debounce and confirms
    immediately when confirmation_threshold=-1 is set.
    """
    # event_type 0 = EVENT_FAILED (a fault occurred); the ReportFault request is
    # {fault_code, event_type, severity, description, source_id}.
    return (
        f"{{source_id: '{source_id}', fault_code: '{fault_code}', "
        f"event_type: {_EVENT_FAILED}, "
        f"severity: {_SEVERITY_CRITICAL}, description: '{description}'}}"
    )


_CALL_TIMEOUT_S = 15


def inject_faults(container: str, n: int, prefix: str = "BENCH_FAULT_",
                  timeout_s: int = _CALL_TIMEOUT_S, cpus: str | None = None) -> int:
    """Report N faults with distinct codes to the fault manager; return how many it ACCEPTED.

    Each fault has a unique code ``<prefix><i>`` at CRITICAL severity, so it
    confirms immediately (no debounce). Every call runs under ``timeout`` inside
    the container, so a missing fault manager costs ``timeout_s`` per fault and
    counts as not accepted instead of hanging. The count comes from the
    ``accepted`` field of the service response. ``cpus`` (a taskset list such as
    "0,1,3") pins the CLI calls.
    """
    pin = f"taskset -c {cpus} " if cpus else ""
    calls = []
    for i in range(1, n + 1):
        yaml_arg = build_inject_yaml(
            source_id=SOURCE_ID,
            fault_code=f"{prefix}{i}",
            description=f"benchmark burst injection {i}",
        )
        calls.append(
            f'out=$({pin}timeout {int(timeout_s)} ros2 service call {_SERVICE} '
            f'{_SERVICE_TYPE} "{yaml_arg}" 2>&1); '
            'case "$out" in *accepted=True*) echo ACCEPTED;; *) echo REJECTED;; esac')
    text = dh.exec_via_file(container, dh.ROS_PRELUDE + "; ".join(calls),
                            timeout=n * (timeout_s + 5) + 30)
    return text.count("ACCEPTED")


def count_rosbag_warns(log_text: str, n_faults: int) -> int:
    """Count how many of the N injected faults DID get a rosbag recording.

    The fault_manager logs a WARN line containing "Already recording post-fault
    data, skipping" when the shared rosbag writer is busy.  Each such WARN means
    one fault was skipped.  The number that DID get a rosbag is n_faults minus
    the skip count.

    Returns the count of faults that received a rosbag (0 <= result <= n_faults).
    This is a pure function - it operates on the raw log text string.
    """
    skip_marker = "Already recording post-fault data, skipping"
    skipped = log_text.count(skip_marker)
    got = max(0, n_faults - skipped)
    return got


def build_fault_rows(
    n_list: List[int],
    modes: List[str],
    burst_results: dict,
    rosbag_counts: dict,
) -> List[dict]:
    """Build a list of row dicts for render_fault_markdown.

    Parameters
    ----------
    n_list:
        List of fault counts (e.g. [1, 2, 4, 8, 16]).
    modes:
        List of mode strings (e.g. ["data", "rosbag"]).
    burst_results:
        Dict keyed by (n, mode) -> summarize_burst result dict.
    rosbag_counts:
        Dict keyed by (n, mode) -> int (number of faults that got rosbag).
        For mode="data" the value is 0 (no rosbag attempted).

    Returns
    -------
    List of row dicts with keys:
        n, mode, status, peak_uss_delta_mib, peak_cpu_cores, capture_duration_s,
        residual_mib, recovered, rosbag_got, rosbag_total.

    A cell whose ``(n, mode)`` key is absent from ``burst_results`` is a FAILED
    cell (it raised during measurement): its row carries ``status="error"`` and
    no measurement fields, so a crashed cell is never rendered or aggregated as a
    genuine zero-valued data point.  Successful cells carry ``status="ok"``.
    """
    rows = []
    for n in n_list:
        for mode in modes:
            key = (n, mode)
            if key not in burst_results:
                rows.append({"n": n, "mode": mode, "status": "error",
                             "rosbag_total": n if mode == "rosbag" else 0})
                continue
            burst = burst_results[key]
            peak_kib = burst.get("peak_uss_delta_kib", 0.0)
            residual_kib = burst.get("residual_uss_delta_kib", 0.0)
            got = rosbag_counts.get(key, 0)
            fm = burst.get("fault_manager", {})
            total = burst.get("sum", {})
            rows.append({
                "n": n,
                "mode": mode,
                "status": "ok",
                "peak_uss_delta_mib": peak_kib / 1024.0,
                "peak_cpu_cores": burst.get("peak_cpu_cores", 0.0),
                "capture_duration_s": burst.get("capture_duration_s", 0.0),
                "residual_mib": residual_kib / 1024.0,
                "recovered": burst.get("recovered", False),
                "rosbag_got": got,
                "rosbag_total": n if mode == "rosbag" else 0,
                "sampler_offset_s": burst.get("sampler_offset_s"),
                "fm_peak_uss_delta_mib": fm.get("peak_uss_delta_kib", 0.0) / 1024.0,
                "fm_peak_cpu_cores": fm.get("peak_cpu_cores", 0.0),
                "fm_settle_s": fm.get("settle_s", 0.0),
                "fm_residual_mib": fm.get("residual_uss_delta_kib", 0.0) / 1024.0,
                "fm_retained": fm.get("retained", False),
                "sum_peak_uss_delta_mib": total.get("peak_uss_delta_kib", 0.0) / 1024.0,
                "sum_peak_cpu_cores": total.get("peak_cpu_cores", 0.0),
            })
    return rows
