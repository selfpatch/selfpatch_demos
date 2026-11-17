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
"""Burst measurement model: sample a transient event (trigger + recovery window).

Pure functions are separated from the live sampler so they can be unit-tested
without docker.
"""
from __future__ import annotations

import csv
import threading
from concurrent.futures import ThreadPoolExecutor
import time
from typing import Callable, Dict, List, Optional, Tuple

from benchmark.lib.metrics import Sample, compute_cpu_cores
from benchmark.lib.sampler import CSV_HEADER, sample_once, write_sample_row


def summarize_burst(
    samples: List[Sample],
    t_trigger: float,
    baseline_uss_kib: float,
    recover_frac: float = 0.05,
    window_s: Optional[float] = None,
    clk_tck: int = 100,
) -> dict:
    """Summarize a burst sample series into scalar stats.

    Parameters
    ----------
    samples:
        Full sample list (pre-trigger baseline + post-trigger window).
    t_trigger:
        Monotonic timestamp at which the trigger fired.
    baseline_uss_kib:
        Baseline USS (KiB) computed by the caller (e.g. median over pre-trigger
        samples).  Must be provided; the function does NOT re-derive it.
    recover_frac:
        Fraction of baseline_uss_kib used as the recovery band.  USS is
        considered recovered when it returns within +/- recover_frac * baseline.
    window_s:
        Total window length in seconds (used as the capture_duration_s upper
        bound when the process never recovers).  When None the actual sample
        span is used as the bound.
    clk_tck:
        Clock ticks per second (getconf CLK_TCK) used to convert /proc CPU ticks
        to cores.  Pass the value resolved from the container so the summarized
        peak CPU matches the per-sample CSV column; defaults to 100 (the common
        Linux value) for callers/tests that do not resolve it.

    Returns
    -------
    dict with keys:
        baseline_uss_kib, peak_uss_delta_kib, peak_cpu_cores,
        capture_duration_s, residual_uss_delta_kib, recovered, settle_s, retained
    """
    post = [s for s in samples if s.t >= t_trigger]
    if not post:
        return {
            "baseline_uss_kib": baseline_uss_kib,
            "peak_uss_delta_kib": 0.0,
            "peak_cpu_cores": 0.0,
            "capture_duration_s": 0.0,
            "residual_uss_delta_kib": 0.0,
            "recovered": True,
        }

    # Peak USS delta (max above baseline after trigger)
    peak_uss_delta_kib = max(s.uss_kib - baseline_uss_kib for s in post)

    # Peak CPU cores: need consecutive pairs; include the last pre-trigger sample
    # as the reference point so the first post-trigger interval has a denominator.
    pre = [s for s in samples if s.t < t_trigger]
    chain = (pre[-1:] if pre else []) + post
    cpu_vals = []
    for prev_s, curr_s in zip(chain, chain[1:]):
        dt = curr_s.t - prev_s.t
        if dt > 0:
            cpu_vals.append(
                compute_cpu_cores(prev_s.total_ticks, curr_s.total_ticks, clk_tck, dt)
            )
    peak_cpu_cores = max(cpu_vals) if cpu_vals else 0.0

    # Recovery: time from trigger until USS returns to within recover_frac of
    # baseline AFTER it has first risen above the band.  Requiring the signal to
    # leave the band first prevents a false instant-recovery when the first
    # post-trigger sample is taken before the burst has ramped (USS still ~=
    # baseline at t_trigger).
    band = recover_frac * baseline_uss_kib
    t_last = post[-1].t
    t_window_end = t_last if window_s is None else t_trigger + window_s

    left_band = False
    recovered_at: Optional[float] = None
    for s in post:
        delta = s.uss_kib - baseline_uss_kib
        if not left_band:
            if delta > band:
                left_band = True
            continue
        if abs(delta) <= band:
            recovered_at = s.t
            break

    if not left_band:
        # USS never measurably left the baseline band: nothing to recover from.
        recovered = True
        capture_duration_s = 0.0
    elif recovered_at is not None:
        recovered = True
        capture_duration_s = recovered_at - t_trigger
    else:
        recovered = False
        capture_duration_s = t_window_end - t_trigger

    # Residual: USS at end of window minus baseline.
    residual_uss_delta_kib = post[-1].uss_kib - baseline_uss_kib

    # Settle: time from the trigger until USS stays within the band of its final
    # value; None when only the last sample is inside it (the series never
    # settled). A process that keeps what the burst created (an in-memory store)
    # never recovers to baseline but does settle.
    tail = 0
    settle_at = post[-1].t
    for s in reversed(post):
        if abs(s.uss_kib - post[-1].uss_kib) > band:
            break
        settle_at = s.t
        tail += 1
    settle_s = settle_at - t_trigger if tail >= 2 else None
    # Retained: USS ends above its baseline by more than the band.
    retained = residual_uss_delta_kib > band

    return {
        "baseline_uss_kib": baseline_uss_kib,
        "peak_uss_delta_kib": peak_uss_delta_kib,
        "peak_cpu_cores": peak_cpu_cores,
        "capture_duration_s": capture_duration_s,
        "residual_uss_delta_kib": residual_uss_delta_kib,
        "recovered": recovered,
        "settle_s": settle_s,
        "retained": retained,
    }


def sample_burst_procs(
    read_fn: Callable[[int, str], str],
    pids: Dict[str, int],
    clk_tck: int,
    host_load_fn: Callable[[], float],
    pre_s: float,
    window_s: float,
    interval: float,
    csv_paths: Dict[str, str],
    trigger_fn: Callable[[], None],
) -> Tuple[Dict[str, List[Sample]], float]:
    """Sample several processes, fire the trigger, keep sampling for window_s.

    The trigger runs on its own thread, so sampling continues while it works
    (injecting N faults takes seconds, and that is when the capture cost
    happens; a trigger that outlasts window_s is sampled to its end, plus one window).
    Returns ({name: samples}, t_trigger); each name's CSV receives its
    rows with per-sample cpu_cores computed incrementally. A trigger exception
    is re-raised after the window.
    """
    samples: Dict[str, List[Sample]] = {name: [] for name in pids}
    prev: Dict[str, Optional[Sample]] = {name: None for name in pids}
    errors: List[BaseException] = []

    def _trigger():
        try:
            trigger_fn()
        except BaseException as exc:  # noqa: BLE001 - re-raised on the caller thread
            errors.append(exc)

    files = {name: open(path, "w", newline="") for name, path in csv_paths.items()}
    try:
        writers = {name: csv.writer(fh) for name, fh in files.items()}
        for w in writers.values():
            w.writerow(CSV_HEADER)

        def _tick():
            load = host_load_fn()
            # Read the processes in parallel so their samples share one instant.
            with ThreadPoolExecutor(max_workers=len(pids)) as pool:
                futs = {name: pool.submit(sample_once, read_fn, pid, load)
                        for name, pid in pids.items()}
                got = {name: f.result() for name, f in futs.items()}
            for name, s in got.items():
                write_sample_row(writers[name], prev[name], s, clk_tck)
                files[name].flush()
                samples[name].append(s)
                prev[name] = s

        end_pre = time.monotonic() + pre_s
        while time.monotonic() < end_pre:
            _tick()
            time.sleep(interval)

        t_trigger = time.monotonic()
        worker = threading.Thread(target=_trigger, daemon=True)
        worker.start()

        # The window is measured from the trigger's start; when the trigger outlasts it,
        # sampling goes on until the trigger ends and then runs for one more window.
        t_done = None
        while True:
            _tick()
            time.sleep(interval)
            now = time.monotonic()
            if worker.is_alive():
                continue
            if t_done is None:
                t_done = now
            end_win = t_done + window_s if t_done > t_trigger + window_s \
                else t_trigger + window_s
            if now >= end_win:
                break
        worker.join()
    finally:
        for fh in files.values():
            fh.close()
    if errors:
        raise errors[0]
    return samples, t_trigger
