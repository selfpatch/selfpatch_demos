"""Jitter lane: cyclictest latency alone and next to a loaded Medkit."""
from __future__ import annotations
import threading

from benchmark.lib import cyclictest
from benchmark.lib import docker_helpers as dh
from benchmark.lib.fault_injector import inject_faults

NON_RT_NOTE = ("This kernel is not PREEMPT_RT: the numbers show the scheduling cost "
               "of Medkit on this host and do not represent a real-time target.")


def _burst_loop(container, burst_n, stop, errors):
    """Inject bursts of distinct faults until ``stop`` is set."""
    n = 0
    while not stop.is_set():
        n += 1
        try:
            inject_faults(container, burst_n, dh.run, prefix=f"JIT{n}_")
        except Exception as exc:  # noqa: BLE001 - recorded, the window keeps running
            errors.append(str(exc))
            return


def run_cyclictest(container, duration_s, cpu):
    out = dh.run(["docker", "exec", container, *cyclictest.build_args(duration_s, cpu)],
                 timeout=duration_s + 120, merge_stderr=True)
    return cyclictest.parse_summary(out), out


def run_load_window(container, duration_s, cpu, burst_n, start_load, stop_load):
    """cyclictest while fault bursts and HTTP clients run in the container."""
    stop = threading.Event()
    errors = []
    start_load()
    worker = threading.Thread(target=_burst_loop, args=(container, burst_n, stop, errors),
                              daemon=True)
    worker.start()
    try:
        result, raw = run_cyclictest(container, duration_s, cpu)
    finally:
        stop.set()
        worker.join(timeout=120)
        stop_load()
    result["load_errors"] = errors
    return result, raw


def _us(v, lower=False):
    return "-" if v is None else f"{'>=' if lower else ''}{v}"


def render_jitter_markdown(rows, meta, params):
    lines = ["# Jitter lane (cyclictest)", ""]
    if not meta.get("preempt_rt"):
        lines += [f"**WARNING: {NON_RT_NOTE}**", ""]
    lines += [
        f"_kernel: {meta.get('kernel')} ({meta.get('kernel_version')}), "
        f"PREEMPT_RT: {'yes' if meta.get('preempt_rt') else 'no'}, "
        f"host: {meta.get('cpu_model')} x{meta.get('nproc')}, "
        f"load1 at end: {meta.get('host_load1')}_", "",
        f"_cyclictest: `{' '.join(params['args'])}` (SCHED_FIFO {cyclictest.PRIORITY}, "
        f"{cyclictest.INTERVAL_US} us interval, one thread on CPU {params['cpu']}, "
        f"{params['duration_s']} s per run, histogram range {cyclictest.HISTOGRAM_US} us)_", "",
        "| scenario | run | min us | avg us | p99 us | max us | samples | overflow |",
        "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append(
            f"| {r['scenario']} | {r['run']} | {r['min_us']} | {r['avg_us']} | "
            f"{_us(r['p99_us'], r['p99_lower_bound'])} | {r['max_us']} | {r['samples']} | "
            f"{r['overflow']} |")
    lines.append("")
    return "\n".join(lines) + "\n"
