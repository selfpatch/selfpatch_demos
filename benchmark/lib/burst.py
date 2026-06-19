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
import time
from typing import Callable, List, Optional, Tuple

from benchmark.lib.metrics import Sample, compute_cpu_cores
from benchmark.lib.sampler import CSV_HEADER, sample_once, write_sample_row


def summarize_burst(
    samples: List[Sample],
    t_trigger: float,
    baseline_uss_kib: float,
    recover_frac: float = 0.05,
    window_s: Optional[float] = None,
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

    Returns
    -------
    dict with keys:
        baseline_uss_kib, peak_uss_delta_kib, peak_cpu_cores,
        capture_duration_s, residual_uss_delta_kib, recovered
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
                compute_cpu_cores(prev_s.total_ticks, curr_s.total_ticks, 100, dt)
            )
    peak_cpu_cores = max(cpu_vals) if cpu_vals else 0.0

    # Recovery: time from trigger until USS is within recover_frac of baseline.
    # "Within band" means |USS - baseline| <= recover_frac * baseline.
    band = recover_frac * baseline_uss_kib
    t_last = post[-1].t
    t_window_end = t_last if window_s is None else t_trigger + window_s

    recovered_at: Optional[float] = None
    for s in post:
        if abs(s.uss_kib - baseline_uss_kib) <= band:
            recovered_at = s.t
            break

    recovered = recovered_at is not None
    if recovered_at is not None:
        capture_duration_s = recovered_at - t_trigger
    else:
        capture_duration_s = t_window_end - t_trigger

    # Residual: USS at end of window minus baseline.
    residual_uss_delta_kib = post[-1].uss_kib - baseline_uss_kib

    return {
        "baseline_uss_kib": baseline_uss_kib,
        "peak_uss_delta_kib": peak_uss_delta_kib,
        "peak_cpu_cores": peak_cpu_cores,
        "capture_duration_s": capture_duration_s,
        "residual_uss_delta_kib": residual_uss_delta_kib,
        "recovered": recovered,
    }


def sample_burst(
    read_fn: Callable[[int, str], str],
    pid: int,
    clk_tck: int,
    host_load_fn: Callable[[], float],
    pre_s: float,
    window_s: float,
    interval: float,
    csv_path: str,
    trigger_fn: Callable[[], None],
) -> Tuple[List[Sample], float]:
    """Sample baseline, fire trigger, then sample for window_s.

    Returns (samples, t_trigger) where samples includes both the pre-trigger
    baseline and the post-trigger window.  The CSV at csv_path receives all
    samples with per-sample cpu_cores computed incrementally.
    """
    samples: List[Sample] = []
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        prev = None

        # Pre-trigger baseline
        end_pre = time.monotonic() + pre_s
        while time.monotonic() < end_pre:
            s = sample_once(read_fn, pid, host_load_fn())
            write_sample_row(w, prev, s, clk_tck)
            fh.flush()
            samples.append(s)
            prev = s
            time.sleep(interval)

        # Trigger
        t_trigger = time.monotonic()
        trigger_fn()

        # Post-trigger window
        end_win = t_trigger + window_s
        while time.monotonic() < end_win:
            s = sample_once(read_fn, pid, host_load_fn())
            write_sample_row(w, prev, s, clk_tck)
            fh.flush()
            samples.append(s)
            prev = s
            time.sleep(interval)

    return samples, t_trigger
