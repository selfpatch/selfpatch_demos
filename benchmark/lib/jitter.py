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

from benchmark.lib import cyclictest
from benchmark.lib import docker_helpers as dh
from benchmark.lib.fault_injector import inject_faults

NON_RT_NOTE = ("This kernel is not PREEMPT_RT: the numbers show the scheduling cost "
               "of Medkit on this host and do not represent a real-time target.")


def _burst_loop(container, burst_n, stop, errors, counter, cpus=None):
    """Inject bursts of distinct faults until ``stop`` is set.

    ``counter`` counts bursts issued and faults the fault manager accepted.
    """
    n = 0
    while not stop.is_set():
        n += 1
        try:
            accepted = inject_faults(container, burst_n, prefix=f"JIT{n}_", cpus=cpus)
        except Exception as exc:  # noqa: BLE001 - recorded, the window keeps running
            errors.append(str(exc))
            return
        counter["bursts"] += 1
        counter["accepted"] += accepted
        if accepted < burst_n:
            errors.append(f"burst {n}: {accepted}/{burst_n} faults accepted")


def run_cyclictest(container, duration_s, cpu):
    out = dh.exec_via_file(container, shlex.join(cyclictest.build_args(duration_s, cpu)),
                           timeout=duration_s + 120)
    return cyclictest.parse_summary(out), out


def load_cpus(container, cpu):
    """CPUs the load generators may use: every CPU of the container except ``cyclictest``'s."""
    n = int(dh.exec_text(container, "nproc").strip())
    return ",".join(str(i) for i in range(n) if i != cpu)


def run_load_window(container, duration_s, cpu, burst_n, start_load, stop_load,
                    read_load_stats, cpus=None):
    """cyclictest while fault bursts and HTTP clients run in the container.

    ``start_load`` returns the path of the load generator's stats file (None when
    it is absent); ``read_load_stats(path)`` returns its stats dict. The result's
    ``load`` holds what the window really ran; a load that did nothing reads 0.
    """
    stop = threading.Event()
    errors = []
    counter = {"bursts": 0, "accepted": 0}
    stats_path = start_load()
    worker = threading.Thread(target=_burst_loop,
                              args=(container, burst_n, stop, errors, counter, cpus),
                              daemon=True)
    worker.start()
    try:
        result, raw = run_cyclictest(container, duration_s, cpu)
    finally:
        stop.set()
        worker.join(timeout=120)
        stop_load()
    stats = read_load_stats(stats_path) or {}
    result["load"] = {
        "load_cpus": cpus,
        "cyclictest_cpu": cpu,
        "fault_bursts": counter["bursts"],
        "faults_requested": counter["bursts"] * burst_n,
        "faults_accepted": counter["accepted"],
        "burst_errors": len(errors),
        "http_requests": int(stats.get("request_count", 0)),
        "http_errors": int(stats.get("error_count", 0)),
    }
    result["load_errors"] = errors
    return result, raw


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
