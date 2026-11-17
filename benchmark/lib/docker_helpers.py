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
import time

# Per-call subprocess timeouts (seconds). A wedged `docker exec` or a gateway
# that accepts a connection but never responds must not hang the whole harness
# indefinitely - every call below is bounded.
_EXEC_TIMEOUT = 60       # short docker exec / ps / inspect calls
_BUILD_TIMEOUT = 3600    # compose up --build (a full ROS image build is slow)
_DOWN_TIMEOUT = 300      # compose down -v


def run(argv, env=None, timeout=None, merge_stderr=False):
    """Run a subprocess, raising RuntimeError on failure or timeout.

    ``timeout`` bounds the call (seconds); None means no bound. ``merge_stderr``
    appends the captured stderr to the returned stdout - required for tools like
    valgrind that write their summary to stderr, which ``docker logs`` forwards
    on the container's stderr stream.
    """
    try:
        res = subprocess.run(argv, capture_output=True, text=True, env=env,
                             timeout=timeout)
    except subprocess.TimeoutExpired:
        raise RuntimeError(f"{' '.join(argv)} timed out after {timeout}s")
    if res.returncode != 0:
        raise RuntimeError(f"{' '.join(argv)} failed: {res.stderr.strip()}")
    return res.stdout + res.stderr if merge_stderr else res.stdout


def compose(args, file, project):
    return ["docker", "compose", "-f", file, "-p", project, *args]


def compose_up(file, project, env=None, service=None):
    # Bring up ONLY the target service (+ its depends_on) so sibling services in
    # the compose file (e.g. a fixed-container-name web UI) don't start/conflict.
    args = ["up", "-d", "--build"]
    if service:
        args.append(service)
    run(compose(args, file, project), env=env, timeout=_BUILD_TIMEOUT)


def compose_down(file, project):
    try:
        run(compose(["down", "-v", "--remove-orphans"], file, project),
            timeout=_DOWN_TIMEOUT)
    except RuntimeError:
        pass


def service_container(file, project, service):
    out = run(compose(["ps", "-q", service], file, project),
              timeout=_EXEC_TIMEOUT).strip()
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
    return parse_pgrep(run(["docker", "exec", container, "pgrep", "-x", proc],
                           timeout=_EXEC_TIMEOUT))


# A process started through `ros2 run` has a python launcher in front of it, and
# `pgrep -x` cannot match names over 15 characters (fault_manager_node has 18).
# Match the full command line: argv[0] must end in the executable name, which the
# launcher's own command line (python3 .../ros2 run <pkg> <exe>) never does.
_EXE_CMDLINE_RE = r"^[^ ]*/{exe}( |$)"

FAULT_MANAGER_EXE = "fault_manager_node"


def resolve_pid_by_exe(container, exe):
    """PID of the single process whose argv[0] ends in ``/<exe>``."""
    out = run(["docker", "exec", container, "pgrep", "-f",
               _EXE_CMDLINE_RE.format(exe=exe)], timeout=_EXEC_TIMEOUT)
    pids = [p for p in out.split() if p.strip()]
    if not pids:
        raise RuntimeError(f"no {exe} process found")
    if len(pids) > 1:
        raise RuntimeError(f"multiple {exe} pids: {pids}")
    return int(pids[0])


def read_cmdline(container, pid):
    """Full command line of a PID, arguments joined by spaces."""
    raw = run(["docker", "exec", container, "cat", f"/proc/{pid}/cmdline"],
              timeout=_EXEC_TIMEOUT)
    return raw.replace("\0", " ").strip()


ROS_PRELUDE = (
    "export ROS_DOMAIN_ID=$(tr '\\0' '\\n' < /proc/1/environ | "
    "sed -n 's/^ROS_DOMAIN_ID=//p'); "
    "source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; ")


def ros_bash(container, script, timeout=_EXEC_TIMEOUT):
    """Run a bash script in the container with the ROS environment sourced.

    `docker exec` does not inherit the compose `environment:`, so the container's
    ROS_DOMAIN_ID is read from PID 1 and exported; without it the command would
    join domain 0 and see no node of the container.
    """
    return run(["docker", "exec", container, "bash", "-c", ROS_PRELUDE + script],
               timeout=timeout)


def exec_via_file(container, script, timeout=_EXEC_TIMEOUT, shell="bash", poll_s=1.0):
    """Run ``script`` in the container and return its stdout and stderr.

    An attached ``docker exec`` can return before a long command ends and lose
    its late output. The script runs detached, writes its output and exit code
    to files, and this call polls for the exit-code file until ``timeout``.
    A non-zero exit code raises RuntimeError.
    """
    out, rc = "/tmp/.bench_exec.out", "/tmp/.bench_exec.rc"
    wrapped = f"rm -f {rc}; ( {script} ) > {out} 2>&1; echo $? > {rc}"
    run(["docker", "exec", container, shell, "-c", f"rm -f {rc} {out}"],
        timeout=_EXEC_TIMEOUT)
    run(["docker", "exec", "-d", container, shell, "-c", wrapped], timeout=_EXEC_TIMEOUT)
    end = time.monotonic() + timeout
    while True:
        res = subprocess.run(["docker", "exec", container, "cat", rc],
                             capture_output=True, text=True, timeout=_EXEC_TIMEOUT)
        if res.returncode == 0 and res.stdout.strip():
            code = res.stdout.strip()
            break
        if time.monotonic() > end:
            raise RuntimeError(f"command did not finish within {timeout}s")
        time.sleep(poll_s)
    text = run(["docker", "exec", container, "cat", out], timeout=_EXEC_TIMEOUT)
    if code != "0":
        raise RuntimeError(f"command exited {code}: {text.strip()[-300:]}")
    return text


def read_proc(container, pid, fname):
    return run(["docker", "exec", container, "cat", f"/proc/{pid}/{fname}"],
               timeout=_EXEC_TIMEOUT)


def exec_text(container, *cmd):
    return run(["docker", "exec", container, *cmd], timeout=_EXEC_TIMEOUT)


def getconf_clk_tck(container):
    """Return the resolved CLK_TCK for the container.

    Consumers thread this value through to compute_cpu_cores, so any positive
    value is handled correctly - CLK_TCK is not guaranteed to be 100 across all
    kernels/architectures, so we only reject non-positive (unusable) values.
    """
    v = int(run(["docker", "exec", container, "getconf", "CLK_TCK"],
                timeout=_EXEC_TIMEOUT).strip())
    if v <= 0:
        raise RuntimeError(f"unusable CLK_TCK={v}")
    return v


def image_digest_of_container(container):
    try:
        return run(["docker", "inspect", "-f", "{{.Image}}", container],
                   timeout=_EXEC_TIMEOUT).strip()
    except RuntimeError:
        return "unknown"


def read_gateway_sha(container):
    """Read gateway_sha written by the Dockerfile build step.

    Tries /ws/gateway_sha (synthetic image) and /root/demo_ws/gateway_sha
    (turtlebot3 demo image) in that order.
    """
    for path in ("/ws/gateway_sha", "/root/demo_ws/gateway_sha"):
        try:
            sha = run(["docker", "exec", container, "cat", path],
                      timeout=_EXEC_TIMEOUT).strip()
            if sha:
                return sha
        except RuntimeError:
            pass
    return "unknown"


def read_allocator_from_maps(container, pid):
    """Classify the memory allocator from /proc/<pid>/maps inside a container."""
    try:
        maps = run(["docker", "exec", container, "cat", f"/proc/{pid}/maps"],
                   timeout=_EXEC_TIMEOUT)
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
    # curl --max-time/--connect-timeout bound the request itself; the subprocess
    # timeout is a slightly larger backstop so a wedged `docker exec` also unwinds.
    out = run(["docker", "exec", container, "curl", "-s",
               "--connect-timeout", "5", "--max-time", "10",
               "-w", "\\n%{http_code}", url], timeout=20)
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
                   f"cat /proc/{pid}/task/*/comm 2>/dev/null || true"],
                  timeout=_EXEC_TIMEOUT)
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
