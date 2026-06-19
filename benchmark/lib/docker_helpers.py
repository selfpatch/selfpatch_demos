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
"""docker/compose wrappers + gateway PID resolution inside a container."""
from __future__ import annotations
import subprocess


def run(argv, env=None):
    res = subprocess.run(argv, capture_output=True, text=True, env=env)
    if res.returncode != 0:
        raise RuntimeError(f"{' '.join(argv)} failed: {res.stderr.strip()}")
    return res.stdout


def compose(args, file, project):
    return ["docker", "compose", "-f", file, "-p", project, *args]


def compose_up(file, project, env=None, service=None):
    # Bring up ONLY the target service (+ its depends_on) so sibling services in
    # the compose file (e.g. a fixed-container-name web UI) don't start/conflict.
    args = ["up", "-d", "--build"]
    if service:
        args.append(service)
    run(compose(args, file, project), env=env)


def compose_down(file, project):
    try:
        run(compose(["down", "-v", "--remove-orphans"], file, project))
    except RuntimeError:
        pass


def service_container(file, project, service):
    out = run(compose(["ps", "-q", service], file, project)).strip()
    if not out:
        raise RuntimeError(f"no container for {service} in {project}")
    return out.splitlines()[0]


def parse_pgrep(out):
    pids = [p for p in out.split() if p.strip()]
    if not pids:
        raise RuntimeError("no gateway_node process found")
    if len(pids) > 1:
        raise RuntimeError(f"multiple gateway_node pids: {pids}")
    return int(pids[0])


def resolve_gateway_pid(container, proc="gateway_node"):
    return parse_pgrep(run(["docker", "exec", container, "pgrep", "-x", proc]))


def read_proc(container, pid, fname):
    return run(["docker", "exec", container, "cat", f"/proc/{pid}/{fname}"])


def exec_text(container, *cmd):
    return run(["docker", "exec", container, *cmd])


def getconf_clk_tck(container):
    v = int(run(["docker", "exec", container, "getconf", "CLK_TCK"]).strip())
    if v != 100:
        raise RuntimeError(f"unexpected CLK_TCK={v} (expected 100)")
    return v


def image_digest_of_container(container):
    try:
        return run(["docker", "inspect", "-f", "{{.Image}}", container]).strip()
    except RuntimeError:
        return "unknown"


def read_gateway_sha(container):
    """Read gateway_sha written by the Dockerfile build step.

    Tries /ws/gateway_sha (synthetic image) and /root/demo_ws/gateway_sha
    (turtlebot3 demo image) in that order.
    """
    for path in ("/ws/gateway_sha", "/root/demo_ws/gateway_sha"):
        try:
            sha = run(["docker", "exec", container, "cat", path]).strip()
            if sha:
                return sha
        except RuntimeError:
            pass
    return "unknown"


def read_allocator_from_maps(container, pid):
    """Classify the memory allocator from /proc/<pid>/maps inside a container."""
    try:
        maps = run(["docker", "exec", container, "cat", f"/proc/{pid}/maps"])
        if "tcmalloc" in maps:
            return "tcmalloc"
        if "jemalloc" in maps:
            return "jemalloc"
        return "glibc"
    except RuntimeError:
        return "unknown"


def curl_status_body(container, url):
    # Probe the gateway from INSIDE the container (DooD: the orchestrator cannot
    # route to the container's bridge IP; published-port localhost also fails).
    out = run(["docker", "exec", container, "curl", "-s", "-w", "\\n%{http_code}", url])
    nl = out.rfind("\n")
    if nl < 0:
        return 0, out
    return int(out[nl + 1:].strip() or 0), out[:nl]


# Thread comm prefix/substring -> category mapping.
#
# Checked in order; first match wins.  Patterns are derived from known Linux
# kernel comm truncation (max 15 chars) and the actual thread names used by:
#   - rclcpp (MultiThreadedExecutor worker threads)
#   - cpp-httplib (thread-pool workers)
#   - FastDDS / rmw_fastrtps (DDS layer threads)
#   - rcutils / ros2 middleware misc threads
#
# rclcpp executor threads: the comm is set to the executable name truncated to
# 15 chars.  For gateway_node that becomes "gateway_node" (12 chars, fits).
# rclcpp also spawns threads whose comms start with "rclcpp".
#
# cpp-httplib thread-pool workers set their comm to "httplib" + suffix via
# set_thread_name in newer builds; older builds inherit the parent comm
# ("gateway_node").  We therefore also match the main-thread comm
# "gateway_node" as a gateway category so those threads are not hidden in
# "other".
#
# DDS: FastDDS names its threads "fastdds_*", "dds_*", "dq*" (delivery queue).
# rmw_cyclonedds uses "cdds*" prefixes.
#
# ROS 2 middleware misc: "rcl_", "intra_process", "waitset".
_THREAD_PREFIXES: list[tuple[str, str]] = [
    # rclcpp executor worker threads
    ("rclcpp", "rclcpp_executor"),
    # cpp-httplib pool threads
    ("httplib", "httplib"),
    # FastDDS delivery queues and I/O threads
    ("fastdds", "dds"),
    ("dds", "dds"),
    ("dq", "dds"),
    # CycloneDDS threads
    ("cdds", "dds"),
    # rcl / intra-process / wait-set infrastructure
    ("rcl_", "rclcpp_executor"),
    ("intra_proc", "rclcpp_executor"),
    ("waitset", "rclcpp_executor"),
    # Plugin infrastructure
    ("plugin", "plugin"),
]

# Exact-match comms that should map to the gateway main process rather than "other".
_GATEWAY_COMMS: frozenset = frozenset({"gateway_node"})


def _categorize_comm(comm: str) -> str:
    """Map a thread comm string to a coarse category.

    Categories:
    - ``rclcpp_executor``: rclcpp executor worker threads and rcl infra
    - ``httplib``: cpp-httplib thread-pool workers
    - ``dds``: FastDDS / CycloneDDS layer threads
    - ``plugin``: plugin loader threads
    - ``gateway``: gateway main/manager threads (comm == "gateway_node")
    - ``other``: everything else
    """
    lower = comm.lower()
    for prefix, category in _THREAD_PREFIXES:
        if lower.startswith(prefix):
            return category
    if lower in _GATEWAY_COMMS:
        return "gateway"
    return "other"


def thread_census(container: str, pid: int) -> dict:
    """Read /proc/<pid>/task/*/comm and group threads by coarse category.

    Returns a dict like::

        {"rclcpp_executor": 4, "httplib": 8, "dds": 16, "plugin": 2,
         "other": 3, "total": 33}

    Reads via a single ``docker exec sh -c "cat /proc/<pid>/task/*/comm"``
    invocation.  Degrades gracefully: returns {"total": 0} on any error.
    """
    try:
        raw = run(["docker", "exec", container, "sh", "-c",
                   f"cat /proc/{pid}/task/*/comm 2>/dev/null || true"])
    except RuntimeError:
        return {"total": 0}

    counts: dict = {}
    total = 0
    for line in raw.splitlines():
        comm = line.strip()
        if not comm:
            continue
        cat = _categorize_comm(comm)
        counts[cat] = counts.get(cat, 0) + 1
        total += 1
    counts["total"] = total
    return counts
