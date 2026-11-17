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
"""Disk lane: install size, fault store growth, capture bytes and log growth.

Parsers and report rendering are pure; the scenario runner drives a container
through the docker helpers.
"""
from __future__ import annotations
import json
import time

from benchmark.lib import docker_helpers as dh
from benchmark.lib.metrics import linfit

_PROBE = "python3 /ws/benchmark/scaler/disk_probe.py"
_FLOOD = "python3 -m benchmark.scaler.fault_flood"
_INSTALL_DIR = "/ws/install"
_PREFIX = "ros2_medkit"

# Left out of the runtime set: what a deployment does not install.
EXCLUDED = ("the ros2_medkit_integration_tests and ros2_medkit_cmake packages",
            "include/ directories (headers)", "static libraries (*.a)",
            "ros2_medkit_opcua/share/open62541 (tool and nodeset data)")

_STRIP = ("find {d} -type f \\( -name '*.so*' -o -name '*.a' -o -perm -u+x \\) "
          "-exec strip --strip-unneeded {{}} + 2>/dev/null || true")


def _du_lines(label, d):
    return (f"cd {d} && for p in */; do printf '{label} %s %s\\n' "
            '"$(du -sb "$p" | cut -f1)" "${p%/}"; done; ')


# One line per (tree, package): "<label> <bytes> <package>". Trees: the install as built,
# the same stripped (every ELF file and static library), the runtime set, and that stripped.
_ALL_CMD = (
    "set -e; rm -rf /tmp/inst_str /tmp/inst_rt /tmp/inst_rtstr; "
    "cp -a /ws/install /tmp/inst_str; " + _STRIP.format(d="/tmp/inst_str") + "; "
    "cp -a /ws/install /tmp/inst_rt; "
    "rm -rf /tmp/inst_rt/ros2_medkit_integration_tests /tmp/inst_rt/ros2_medkit_cmake "
    "/tmp/inst_rt/ros2_medkit_opcua/share/open62541; "
    "find /tmp/inst_rt -type d -name include -prune -exec rm -rf {} +; "
    "find /tmp/inst_rt -name '*.a' -delete; "
    "cp -a /tmp/inst_rt /tmp/inst_rtstr; " + _STRIP.format(d="/tmp/inst_rtstr") + "; " +
    _du_lines("full", _INSTALL_DIR) + _du_lines("stripped", "/tmp/inst_str") +
    _du_lines("runtime", "/tmp/inst_rt") + _du_lines("runtime_stripped", "/tmp/inst_rtstr") +
    "rm -rf /tmp/inst_str /tmp/inst_rt /tmp/inst_rtstr")


def parse_install_sizes(text):
    """Parse ``<label> <bytes> <package>`` lines into {label: {package: bytes}}."""
    out = {}
    for line in text.splitlines():
        parts = line.split(None, 2)
        if len(parts) == 3 and parts[1].isdigit():
            out.setdefault(parts[0], {})[parts[2].strip()] = int(parts[1])
    return out


def install_report(trees):
    """Medkit packages (name prefix ros2_medkit) with their sizes in each tree and the totals."""
    full = trees.get("full", {})
    pkgs = sorted(k for k in full if k.startswith(_PREFIX))
    labels = ("full", "stripped", "runtime", "runtime_stripped")
    rows = [{"package": k, **{lb: trees.get(lb, {}).get(k) for lb in labels}} for k in pkgs]
    out = {"packages": rows, "excluded": list(EXCLUDED),
           "other_bytes": sum(v for k, v in full.items() if k not in pkgs)}
    for lb in labels:
        out[f"{lb}_bytes"] = sum(trees.get(lb, {}).get(k, 0) for k in pkgs)
    return out


def measure_install(container):
    return install_report(parse_install_sizes(
        dh.exec_via_file(container, _ALL_CMD, timeout=900, shell="sh")))


def probe(container):
    return json.loads(dh.exec_text(container, "sh", "-c", _PROBE))


def wait_probe(container, predicate, timeout_s=120, poll_s=1.0):
    """Probe until ``predicate(probe)`` holds; return the probe, or raise."""
    end = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < end:
        try:
            last = probe(container)
        except Exception:
            last = None
        if last is not None and predicate(last):
            return last
        time.sleep(poll_s)
    raise RuntimeError(f"disk probe condition not met within {timeout_s}s: {last}")


def _stable(prev, cur):
    return (prev is not None and prev["rosbag_bytes"] == cur["rosbag_bytes"]
            and prev["tables"].get("rosbag_files") == cur["tables"].get("rosbag_files"))


def _log_settled(prev, cur):
    return prev is not None and prev["log_bytes"] == cur["log_bytes"]


def wait_started(container, timeout_s=180, quiet_probes=5, poll_s=2.0):
    """Probe until the fault store exists and the log directory stopped growing.

    The services write a start-up burst of log lines; waiting until the log size
    holds for ``quiet_probes`` consecutive probes keeps that flush out of every
    growth value.
    """
    end = time.monotonic() + timeout_s
    prev, quiet, last = None, 0, None
    while time.monotonic() < end:
        try:
            last = probe(container)
        except Exception:
            last = None
        if last is not None and "faults" in last["tables"]:
            quiet = quiet + 1 if _log_settled(prev, last) else 0
            if quiet >= quiet_probes:
                return last
        prev = last
        time.sleep(poll_s)
    raise RuntimeError(f"fault store or log never settled within {timeout_s}s: {last}")


def run_scenario(compose_file, project, service, env, flood_args=None,
                 idle_s=0.0, expect_rosbags=0, ready_timeout=180, window_s=0.0):
    """One fresh container: baseline probe, optional fault flood or idle window, final probe.

    ``flood_args`` is the fault_flood command line (without the program).
    ``expect_rosbags`` makes the final probe wait for that many recording rows
    and for the recording directory size to stop changing.
    ``window_s`` is the shortest time between the two probes, so every scenario
    measures growth over the same window.
    Returns a dict with both probes, the flood result and the growth values.
    """
    dh.compose_down(compose_file, project)
    dh.compose_up(compose_file, project, env=env, service=service)
    try:
        container = dh.service_container(compose_file, project, service)
        before = wait_started(container, timeout_s=ready_timeout)
        t0 = time.monotonic()
        flood = None
        if flood_args:
            raw = dh.exec_via_file(
                container, dh.ROS_PRELUDE + f"cd /ws && {_FLOOD} {flood_args}",
                timeout=3600)
            flood = json.loads(
                [ln for ln in raw.splitlines() if ln.startswith("{")][-1])
            if "error" in flood:
                raise RuntimeError(f"fault flood failed: {flood['error']}")
        # The measurement window itself: growth is taken over this much time.
        rest = max(idle_s, window_s) - (time.monotonic() - t0)
        if rest > 0:
            time.sleep(rest)
        if expect_rosbags:
            prev = None

            def settled(p):
                nonlocal prev
                ok = p["tables"].get("rosbag_files", 0) >= expect_rosbags and _stable(prev, p)
                prev = p
                return ok
            after = wait_probe(container, settled, timeout_s=120, poll_s=2.0)
        else:
            after = probe(container)
        return {"before": before, "after": after, "flood": flood,
                "elapsed_s": time.monotonic() - t0}
    finally:
        dh.compose_down(compose_file, project)


def _file_growth(a, b, names):
    fa, fb = a["store_files"], b["store_files"]
    return sum(fa.get(n, 0) - fb.get(n, 0) for n in names)


def growth(result):
    """Growth values of one scenario (after minus before), in bytes."""
    b, a = result["before"], result["after"]
    accepted = (result.get("flood") or {}).get("accepted", 0)
    out = {
        "store_bytes": a["store_bytes"] - b["store_bytes"],
        "db_file_bytes": _file_growth(a, b, ("faults.db",)),
        "wal_bytes": _file_growth(a, b, ("faults.db-wal", "faults.db-shm")),
        "db_used_bytes": a["db"].get("db_used_bytes", 0) - b["db"].get("db_used_bytes", 0),
        "rosbag_bytes": a["rosbag_bytes"] - b["rosbag_bytes"],
        "recordings": a["rosbag_entries"] - b["rosbag_entries"],
        "log_bytes": a["log_bytes"] - b["log_bytes"],
        "elapsed_s": result["elapsed_s"],
        "accepted": accepted,
        "tables_after": a["tables"],
    }
    tb_a, tb_b = a["db"].get("table_bytes", {}), b["db"].get("table_bytes", {})
    out["table_growth_bytes"] = {k: tb_a[k] - tb_b.get(k, 0) for k in tb_a
                                 if tb_a[k] - tb_b.get(k, 0)}
    out["db_bytes_per_fault"] = (out["db_used_bytes"] / accepted) if accepted else None
    out["store_bytes_per_fault"] = (out["store_bytes"] / accepted) if accepted else None
    out["rosbag_bytes_per_fault"] = (out["rosbag_bytes"] / accepted) if accepted else None
    return out


def marginal_bytes_per_fault(points):
    """Least-squares slope of db bytes against fault count over (count, bytes) points.

    Needs at least two distinct counts; returns None otherwise.
    """
    pts = [(n, b) for n, b in points if n is not None]
    if len({n for n, _ in pts}) < 2:
        return None
    return linfit([float(n) for n, _ in pts], [float(b) for _, b in pts])[0]


def _kib(b):
    return f"{b / 1024:.1f} KiB"


def _mib(b):
    return f"{b / 1024 / 1024:.2f} MiB"


def render_disk_markdown(report, meta=None):
    lines = ["# Disk lane", ""]
    if meta:
        lines += [f"_host: {meta.get('cpu_model')} x{meta.get('nproc')}, "
                  f"kernel {meta.get('kernel')}_", ""]
    inst = report.get("install")
    if inst:
        lines += ["## Installed size of the Medkit packages", "",
                  f"_{report.get('build_note', '')}_", "",
                  "| package | full install | full, stripped | runtime set | runtime set, stripped |",
                  "|---|---|---|---|---|"]

        def cell(v):
            return _mib(v) if v is not None else "-"
        for r in inst["packages"]:
            lines.append(f"| {r['package']} | {cell(r['full'])} | {cell(r['stripped'])} | "
                         f"{cell(r['runtime'])} | {cell(r['runtime_stripped'])} |")
        lines += [f"| **total (ros2_medkit*)** | **{_mib(inst['full_bytes'])}** | "
                  f"**{_mib(inst['stripped_bytes'])}** | **{_mib(inst['runtime_bytes'])}** | "
                  f"**{_mib(inst['runtime_stripped_bytes'])}** |", "",
                  "_Stripped = `strip --strip-unneeded` on every ELF file and static library. "
                  "The runtime set leaves out: " + "; ".join(inst["excluded"]) + "._",
                  f"_Other packages in the install tree: {_mib(inst['other_bytes'])}._", ""]
    d = report.get("distinct")
    if d:
        lines += ["## Fault store growth, distinct faults (default storage, snapshots on)", "",
                  "| faults | accepted | db used (logical) | db file | WAL + shm | "
                  "total on disk | total per fault | rosbag | log growth | "
                  "freeze_frames | snapshots |", "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in d:
            g = r["growth"]
            per = (_kib(g["store_bytes_per_fault"])
                   if g["store_bytes_per_fault"] is not None else "-")
            lines.append(
                f"| {r['n']} | {g['accepted']} | {_kib(g['db_used_bytes'])} | "
                f"{_kib(g['db_file_bytes'])} | {_kib(g['wal_bytes'])} | "
                f"{_kib(g['store_bytes'])} | {per} | "
                f"{_kib(g['rosbag_bytes'])} | {_kib(g['log_bytes'])} | "
                f"{g['tables_after'].get('freeze_frames', '-')} | "
                f"{g['tables_after'].get('snapshots', '-')} |")
        slope = report.get("distinct_marginal_bytes")
        lines += ["", "_`total on disk` is the size of the store files (db, WAL, shm). The "
                      "fault manager keeps the WAL at its high-water size while it runs, so "
                      "total can exceed the logical db size by the WAL._"]
        if slope is not None:
            lines += ["", f"_Marginal logical store cost: {_kib(slope)} per additional "
                          "distinct fault (least-squares slope over the `db used` column)._"]
        lines.append("")
    o = report.get("occurrences")
    if o:
        lines += ["## Fault store growth, occurrences of one fault code", "",
                  "| variant | occurrences | accepted | db used (logical) | total on disk | "
                  "snapshots rows | capture ids | faults rows |", "|---|---|---|---|---|---|---|---|"]
        for r in o:
            g = r["growth"]
            t = g["tables_after"]
            lines.append(
                f"| {r['variant']} | {r['n']} | {g['accepted']} | {_kib(g['db_used_bytes'])} | "
                f"{_kib(g['store_bytes'])} | {t.get('snapshots', '-')} | {t.get('max_capture_id', '-')} | "
                f"{t.get('faults', '-')} |")
        lines += ["", "_`snapshots.max_per_fault` (default 10) caps the snapshot sets kept per "
                      "code. A clear drops the code's cooldown entry, so "
                      "`snapshots.recapture_cooldown_sec` (default 60) does not limit these "
                      "clear-and-reraise cycles; it gates re-confirmation without a clear._", ""]
    rb = report.get("rosbag")
    if rb:
        lines += ["## Rosbag capture (rosbag enabled, other settings default)", "",
                  "| scenario | faults | recordings (bag files) | rosbag bytes | per fault |",
                  "|---|---|---|---|---|"]
        for r in rb:
            g = r["growth"]
            per = _kib(g["rosbag_bytes_per_fault"]) if g["rosbag_bytes_per_fault"] is not None else "-"
            lines.append(f"| {r['variant']} | {g['accepted']} | "
                         f"{g['recordings']} | "
                         f"{_kib(g['rosbag_bytes'])} | {per} |")
        lines.append("")
    idle = report.get("idle")
    if idle:
        g = idle["growth"]
        rate = g["log_bytes"] / g["elapsed_s"] * 3600 if g["elapsed_s"] else 0
        lines += ["## ROS log directory, no faults", "",
                  f"Over {g['elapsed_s']:.0f} s with gateway, fault manager and one publisher "
                  f"running, after the start-up log flush: store growth {g['store_bytes']} B, "
                  f"log growth {g['log_bytes']} B (rate {_kib(rate)} per hour).", ""]
    return "\n".join(lines) + "\n"
