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
"""Jitter lane: cyclictest latency alone and next to a loaded Medkit."""
from __future__ import annotations
import shlex
import threading
import time

from benchmark.lib import cyclictest
from benchmark.lib import docker_helpers as dh
from benchmark.lib.fault_injector import inject_faults

NON_RT_NOTE = ("This kernel is not PREEMPT_RT: the numbers show the scheduling cost "
               "of Medkit on this host and do not represent a real-time target.")


def _burst_loop(container, burst_n, stop, errors, bursts, cpus=None):
    """Inject bursts of distinct faults until ``stop`` is set.

    ``bursts`` collects one (start, end, accepted) tuple per burst, in monotonic seconds.
    """
    n = 0
    while not stop.is_set():
        n += 1
        t0 = time.monotonic()
        try:
            accepted = inject_faults(container, burst_n, prefix=f"JIT{n}_", cpus=cpus)
        except Exception as exc:  # noqa: BLE001 - recorded, the window keeps running
            errors.append(str(exc))
            return
        bursts.append((t0, time.monotonic(), accepted))
        if accepted < burst_n:
            errors.append(f"burst {n}: {accepted}/{burst_n} faults accepted")


def run_cyclictest(container, duration_s, cpu):
    check_affinity(int(dh.exec_text(container, "nproc").strip()), cpu)
    out = dh.exec_via_file(container, shlex.join(cyclictest.build_args(duration_s, cpu)),
                           timeout=duration_s + 120)
    return cyclictest.parse_summary(out), out


def check_affinity(nproc, cpu):
    """Refuse an affinity cyclictest cannot honour or that leaves no CPU for the load."""
    if nproc < 2:
        raise ValueError(f"jitter needs at least 2 CPUs (one for cyclictest, one for the "
                         f"load); the container has {nproc}")
    if not 0 <= cpu < nproc:
        raise ValueError(f"cyclictest CPU {cpu} is outside 0..{nproc - 1}")


def load_cpus(container, cpu):
    """CPUs the load generators may use: every CPU of the container except ``cyclictest``'s."""
    n = int(dh.exec_text(container, "nproc").strip())
    return ",".join(str(i) for i in range(n) if i != cpu)


def run_load_window(container, duration_s, cpu, burst_n, start_load, stop_load,
                    read_load_stats, cpus=None, read_progress=None):
    """cyclictest while fault bursts and HTTP clients run in the container.

    ``start_load`` returns the path of the load generator's stats file (None when
    it is absent); ``read_load_stats(path)`` returns its stats dict;
    ``read_progress()`` returns the generator's running {"t", "request_count",
    "error_count"} (wall time) or None. The result's ``load`` counts only work inside
    the cyclictest window: a fault burst counts when it starts and ends inside it,
    HTTP requests are the difference of two progress reads taken at its edges.
    """
    check_affinity(int(dh.exec_text(container, "nproc").strip()), cpu)
    stop = threading.Event()
    errors = []
    bursts = []
    stats_path = start_load()
    worker = threading.Thread(target=_burst_loop,
                              args=(container, burst_n, stop, errors, bursts, cpus),
                              daemon=True)
    worker.start()
    p0 = read_progress() if read_progress else None
    t0 = time.monotonic()
    try:
        result, raw = run_cyclictest(container, duration_s, cpu)
    finally:
        t1 = time.monotonic()
        p1 = read_progress() if read_progress else None
        stop.set()
        worker.join(timeout=120)
        stop_load()
    stats = read_load_stats(stats_path) or {}
    inside = [b for b in list(bursts) if b[0] >= t0 and b[1] <= t1]
    window_s = t1 - t0
    http = {"requests": None, "errors": None, "window_s": None}
    if p0 and p1:
        http = {"requests": p1["request_count"] - p0["request_count"],
                "errors": p1["error_count"] - p0["error_count"],
                "window_s": round(p1["t"] - p0["t"], 2)}
    result["load"] = {
        "load_cpus": cpus,
        "cyclictest_cpu": cpu,
        "window_s": round(window_s, 2),
        "counted": "inside the cyclictest window: bursts that start and end in it; HTTP "
                   "requests between two progress reads at its edges",
        "fault_bursts": len(inside),
        "faults_requested": len(inside) * burst_n,
        "faults_accepted": sum(b[2] for b in inside),
        "faults_per_s": round(len(inside) * burst_n / window_s, 3) if window_s else None,
        "burst_errors": len(errors),
        "http_requests": http["requests"] if http["requests"] is not None
        else int(stats.get("request_count", 0)),
        "http_errors": http["errors"] if http["errors"] is not None
        else int(stats.get("error_count", 0)),
        "http_per_s": round(http["requests"] / http["window_s"], 2)
        if http["requests"] is not None and http["window_s"] else None,
        "http_window_s": http["window_s"],
    }
    result["load_errors"] = errors
    return result, raw


# A loaded row's request rates must match the other loaded row within this share.
LOAD_TOLERANCE = 0.25


def check_load_parity(a, b, tol=LOAD_TOLERANCE):
    """Problems (a list of strings) when two loaded rows did not run the same load.

    Compared: the fault bursts completed in the window and the HTTP request rate, within
    ``tol`` of the larger one (bursts: or one burst), every fault accepted, and the HTTP
    error share, which must stay under 5% in both.
    """
    problems = []
    la, lb = a["load"], b["load"]
    # Bursts are whole units: one burst more or less is within the tolerance.
    nx, ny = la["fault_bursts"], lb["fault_bursts"]
    if abs(nx - ny) > max(1, tol * max(nx, ny)):
        problems.append(f"fault bursts inside the window differ by more than {tol:.0%} "
                        f"or one burst: {nx} vs {ny}")
    x, y = la.get("http_per_s"), lb.get("http_per_s")
    if x is None or y is None:
        problems.append("HTTP request rate not measured")
    elif abs(x - y) > tol * max(x, y):
        problems.append(f"HTTP request rates differ by more than {tol:.0%}: {x} vs {y} per s")
    for row in (la, lb):
        if row["fault_bursts"] and row["faults_accepted"] < row["faults_requested"]:
            problems.append(f"faults rejected ({row['faults_accepted']}/{row['faults_requested']})")
    for row in (la, lb):
        n = row.get("http_requests") or 0
        if n and (row.get("http_errors") or 0) / n > 0.05:
            problems.append(f"HTTP error share above 5% ({row['http_errors']}/{n})")
    return problems


def _us(v, lower=False):
    return "-" if v is None else f"{'>=' if lower else ''}{v}"


def _load_cells(row):
    ld = row.get("load")
    if ld is None:
        return "- | -"
    errs = ld["burst_errors"] + ld["http_errors"]
    return (f"{ld['fault_bursts']} ({ld['faults_accepted']}/{ld['faults_requested']} "
            f"faults accepted) | "
            f"{ld['http_requests']} ({errs} errors)")


def render_jitter_markdown(rows, meta, params):
    lines = ["# Jitter lane (cyclictest)", ""]
    if not meta.get("preempt_rt"):
        lines += [f"**WARNING: {NON_RT_NOTE}**", ""]
    if meta.get("high_host_load"):
        lines += [f"**WARNING: collected under high host load (load1={meta.get('host_load1')} > "
                  f"{meta.get('nproc')} cores); other work on the host moves the baseline "
                  "too, so latency differences are not attributable to Medkit.**", ""]
    from benchmark.lib.report import process_meta_lines
    lines += process_meta_lines(meta)
    lines += [
        f"_kernel: {meta.get('kernel')} ({meta.get('kernel_version')}), "
        f"PREEMPT_RT: {'yes' if meta.get('preempt_rt') else 'no'}, "
        f"host: {meta.get('cpu_model')} x{meta.get('nproc')}, "
        f"load1 at end: {meta.get('host_load1')}_", "",
        f"_cyclictest: `{' '.join(params['args'])}` (SCHED_FIFO {cyclictest.PRIORITY}, "
        f"{cyclictest.INTERVAL_US} us interval, one thread on CPU {params['cpu']}, "
        f"{params['duration_s']} s per run, histogram range {cyclictest.HISTOGRAM_US} us)_", "",
        f"_load generators (fault bursts, HTTP clients) are pinned to CPUs "
        f"{params.get('load_cpus', 'n/a')}, not to cyclictest's CPU; the fault manager runs "
        "with its in-memory store (no SQLite writes in this lane)._", "",
        "| scenario | run | min us | avg us | p99 us | max us | samples | overflow | "
        "fault bursts | HTTP requests (errors) |",
        "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(
            f"| {r['scenario']} | {r['run']} | {r['min_us']} | {r['avg_us']} | "
            f"{_us(r['p99_us'], r['p99_lower_bound'])} | {r['max_us']} | {r['samples']} | "
            f"{r['overflow']} | {_load_cells(r)} |")
    lines.append("")
    return "\n".join(lines) + "\n"
