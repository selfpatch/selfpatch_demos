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


# ReportFault.srv request fields (verified against ros2_medkit_msgs source):
#   string source_id
#   string fault_code
#   uint8 severity   (INFO=0, WARNING=1, ERROR=2, CRITICAL=3)
#   string description
_SEVERITY_CRITICAL = 3
_EVENT_FAILED = 0
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


def inject_faults(container: str, n: int, run_fn) -> None:
    """Inject N faults with distinct codes into the fault_manager via docker exec.

    Each fault has a unique code ``BENCH_FAULT_<i>`` and is reported at
    CRITICAL severity so it confirms immediately (no debounce).

    ``run_fn`` is a callable with the same signature as ``docker_helpers.run``:
    it accepts a list-of-str argv and raises RuntimeError on failure.

    Uses ``ros2 service call`` inside the container, sourcing the ROS and
    workspace install envs first.
    """
    for i in range(1, n + 1):
        fault_code = f"BENCH_FAULT_{i}"
        yaml_arg = build_inject_yaml(
            source_id="benchmark",
            fault_code=fault_code,
            description=f"benchmark burst injection {i}",
        )
        cmd = [
            "docker", "exec", container,
            "bash", "-c",
            (
                # docker exec does NOT inherit the compose `environment:` vars, so
                # ROS_DOMAIN_ID would default to 0 and the injector's transient node
                # would not discover the fault_manager (which runs on the container's
                # domain). Read PID 1's ROS_DOMAIN_ID so we join the same domain.
                "export ROS_DOMAIN_ID=$(tr '\\0' '\\n' < /proc/1/environ | "
                "sed -n 's/^ROS_DOMAIN_ID=//p'); "
                "source /opt/ros/jazzy/setup.bash; "
                "source /ws/install/setup.bash; "
                f"ros2 service call {_SERVICE} {_SERVICE_TYPE} "
                f'"{yaml_arg}"'
            ),
        ]
        run_fn(cmd)


def count_rosbag_warns(log_text: str, n_faults: int) -> int:
    """Count how many of the N injected faults did NOT get a rosbag recording.

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
        n, mode, peak_uss_delta_mib, peak_cpu_cores, capture_duration_s,
        residual_mib, recovered, rosbag_got, rosbag_total.
    """
    rows = []
    for n in n_list:
        for mode in modes:
            key = (n, mode)
            burst = burst_results.get(key, {})
            peak_kib = burst.get("peak_uss_delta_kib", 0.0)
            residual_kib = burst.get("residual_uss_delta_kib", 0.0)
            got = rosbag_counts.get(key, 0)
            rows.append({
                "n": n,
                "mode": mode,
                "peak_uss_delta_mib": peak_kib / 1024.0,
                "peak_cpu_cores": burst.get("peak_cpu_cores", 0.0),
                "capture_duration_s": burst.get("capture_duration_s", 0.0),
                "residual_mib": residual_kib / 1024.0,
                "recovered": burst.get("recovered", False),
                "rosbag_got": got,
                "rosbag_total": n if mode == "rosbag" else 0,
            })
    return rows
