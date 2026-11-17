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

# One line per install directory: "<bytes> <name>" (apparent size, symlinks not followed).
_INSTALL_CMD = (
    f"cd {_INSTALL_DIR} && for d in */; do "
    'printf "%s %s\\n" "$(du -sb "$d" | cut -f1)" "${d%/}"; done')

# Copy, strip ELF files in the copy, then size it: what a stripped image would carry.
_STRIPPED_CMD = (
    f"rm -rf /tmp/inst_stripped && cp -a {_INSTALL_DIR} /tmp/inst_stripped && "
    "find /tmp/inst_stripped -type f \\( -name '*.so*' -o -perm -u+x \\) "
    "-exec strip --strip-unneeded {} + 2>/dev/null; " +
    _INSTALL_CMD.replace(_INSTALL_DIR, "/tmp/inst_stripped"))


def parse_install_sizes(text):
    """Parse ``<bytes> <package>`` lines into {package: bytes}."""
    out = {}
    for line in text.splitlines():
        parts = line.split(None, 1)
        if len(parts) == 2 and parts[0].isdigit():
            out[parts[1].strip()] = int(parts[0])
    return out


def install_report(sizes, stripped):
    """Medkit packages (name prefix ros2_medkit) with their sizes and the totals."""
    pkgs = sorted(k for k in sizes if k.startswith(_PREFIX))
    rows = [{"package": k, "bytes": sizes[k], "stripped_bytes": stripped.get(k)}
            for k in pkgs]
    return {
        "packages": rows,
        "medkit_bytes": sum(sizes[k] for k in pkgs),
        "medkit_stripped_bytes": sum(stripped.get(k, 0) for k in pkgs),
        "other_bytes": sum(v for k, v in sizes.items() if k not in pkgs),
    }


def measure_install(container):
    sizes = parse_install_sizes(dh.exec_text(container, "sh", "-c", _INSTALL_CMD))
    stripped = parse_install_sizes(
        dh.exec_via_file(container, _STRIPPED_CMD, timeout=600, shell="sh"))
    return install_report(sizes, stripped)


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


def run_scenario(compose_file, project, service, env, flood_args=None,
                 idle_s=0.0, expect_rosbags=0, ready_timeout=180):
    """One fresh container: baseline probe, optional fault flood or idle window, final probe.

    ``flood_args`` is the fault_flood command line (without the program).
    ``expect_rosbags`` makes the final probe wait for that many recording rows
    and for the recording directory size to stop changing.
    Returns a dict with both probes, the flood result and the growth values.
    """
    dh.compose_down(compose_file, project)
    dh.compose_up(compose_file, project, env=env, service=service)
    try:
        container = dh.service_container(compose_file, project, service)
        before = wait_probe(container, lambda p: "faults" in p["tables"],
                            timeout_s=ready_timeout)
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
        if idle_s:
            time.sleep(idle_s)
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


def growth(result):
    """Growth values of one scenario (after minus before), in bytes."""
    b, a = result["before"], result["after"]
    accepted = (result.get("flood") or {}).get("accepted", 0)
    out = {
        "store_bytes": a["store_bytes"] - b["store_bytes"],
        "db_used_bytes": a["db"].get("db_used_bytes", 0) - b["db"].get("db_used_bytes", 0),
        "rosbag_bytes": a["rosbag_bytes"] - b["rosbag_bytes"],
        "log_bytes": a["log_bytes"] - b["log_bytes"],
        "elapsed_s": result["elapsed_s"],
        "accepted": accepted,
        "tables_after": a["tables"],
    }
    tb_a, tb_b = a["db"].get("table_bytes", {}), b["db"].get("table_bytes", {})
    out["table_growth_bytes"] = {k: tb_a[k] - tb_b.get(k, 0) for k in tb_a
                                 if tb_a[k] - tb_b.get(k, 0)}
    out["db_bytes_per_fault"] = (out["db_used_bytes"] / accepted) if accepted else None
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
                  "| package | installed | stripped |", "|---|---|---|"]
        for r in inst["packages"]:
            st = _mib(r["stripped_bytes"]) if r["stripped_bytes"] is not None else "-"
            lines.append(f"| {r['package']} | {_mib(r['bytes'])} | {st} |")
        lines += [f"| **total (ros2_medkit*)** | **{_mib(inst['medkit_bytes'])}** | "
                  f"**{_mib(inst['medkit_stripped_bytes'])}** |", "",
                  f"_Other packages in the install tree: {_mib(inst['other_bytes'])}._", ""]
    d = report.get("distinct")
    if d:
        lines += ["## Fault store growth, distinct faults (default storage, snapshots on)", "",
                  "| faults | accepted | db used | per fault | rosbag | log growth | "
                  "freeze_frames | snapshots |", "|---|---|---|---|---|---|---|---|"]
        for r in d:
            g = r["growth"]
            per = _kib(g["db_bytes_per_fault"]) if g["db_bytes_per_fault"] is not None else "-"
            lines.append(
                f"| {r['n']} | {g['accepted']} | {_kib(g['db_used_bytes'])} | {per} | "
                f"{_kib(g['rosbag_bytes'])} | {_kib(g['log_bytes'])} | "
                f"{g['tables_after'].get('freeze_frames', '-')} | "
                f"{g['tables_after'].get('snapshots', '-')} |")
        slope = report.get("distinct_marginal_bytes")
        if slope is not None:
            lines += ["", f"_Marginal store cost: {_kib(slope)} per additional distinct fault "
                          "(least-squares slope over the rows above)._"]
        lines.append("")
    o = report.get("occurrences")
    if o:
        lines += ["## Fault store growth, occurrences of one fault code", "",
                  "| variant | occurrences | accepted | db used | snapshots rows | "
                  "capture ids | faults rows |", "|---|---|---|---|---|---|---|"]
        for r in o:
            g = r["growth"]
            t = g["tables_after"]
            lines.append(
                f"| {r['variant']} | {r['n']} | {g['accepted']} | {_kib(g['db_used_bytes'])} | "
                f"{t.get('snapshots', '-')} | {t.get('max_capture_id', '-')} | "
                f"{t.get('faults', '-')} |")
        lines += ["", "_`snapshots.max_per_fault` (default 10) caps the snapshot sets kept per "
                      "code. A clear drops the code's cooldown entry, so "
                      "`snapshots.recapture_cooldown_sec` (default 60) does not limit these "
                      "clear-and-reraise cycles; it gates re-confirmation without a clear._", ""]
    rb = report.get("rosbag")
    if rb:
        lines += ["## Rosbag capture (rosbag enabled, other settings default)", "",
                  "| scenario | faults | recordings | rosbag bytes | per fault |",
                  "|---|---|---|---|---|"]
        for r in rb:
            g = r["growth"]
            per = _kib(g["rosbag_bytes_per_fault"]) if g["rosbag_bytes_per_fault"] is not None else "-"
            lines.append(f"| {r['variant']} | {g['accepted']} | "
                         f"{g['tables_after'].get('rosbag_files', '-')} | "
                         f"{_kib(g['rosbag_bytes'])} | {per} |")
        lines.append("")
    idle = report.get("idle")
    if idle:
        g = idle["growth"]
        rate = g["log_bytes"] / g["elapsed_s"] * 3600 if g["elapsed_s"] else 0
        lines += ["## ROS log directory, no faults", "",
                  f"Over {g['elapsed_s']:.0f} s with gateway, fault manager and one publisher "
                  f"running: store growth {g['store_bytes']} B, log growth {g['log_bytes']} B "
                  f"(measured rate {_kib(rate)} per hour).", ""]
    return "\n".join(lines) + "\n"
