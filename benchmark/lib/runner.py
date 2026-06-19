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
"""Shared cell runner: fresh container, warmup gate, sample window, summarize."""
from __future__ import annotations
import json
import os
import time

from benchmark.lib import docker_helpers as dh
from benchmark.lib import gateway_client, sampler, warmup
from benchmark.lib.metrics import median, iqr, peak_uss, slope_ci95, steady_window

_LOAD_STATS_PATH = "/tmp/load_stats.json"

# Steady-window flatness threshold: if the absolute USS slope exceeds this value
# (in B/s) AND the 95% CI excludes zero, the window is flagged not-steady.
_STEADY_SLOPE_THRESHOLD_B_S = 1024.0


def summarize_window(samples, clk_tck):
    """Summarize a sample window into scalar stats.

    ``samples`` is the full list of samples; stats are computed over the
    steady window (last 1/3).  The returned dict carries:
      - ``samples``: length of the steady window used for statistics
      - ``samples_total``: total length of the raw sample list
    """
    win = steady_window(samples)
    if not win:
        return {"samples": 0, "samples_total": len(samples),
                "status": "failed", "error": "no samples"}
    uss = [s.uss_kib for s in win]
    cores = sampler.per_sample_cores(win, clk_tck) or [0.0]
    slope = slope_ci95([s.t for s in win], [s.uss_kib * 1024 for s in win])
    wlo, whi = iqr(uss)
    return {
        "uss_kib": median(uss), "pss_kib": median([s.pss_kib for s in win]),
        "rss_kib": median([s.rss_kib for s in win]),
        "cpu_cores": median(cores),
        "peak_uss_kib": peak_uss(samples),
        "num_threads": median([s.num_threads for s in win]),
        "uss_within_iqr_lo": wlo, "uss_within_iqr_hi": whi,
        "uss_slope_b_s": slope[0], "uss_slope_lo95": slope[1], "uss_slope_hi95": slope[2],
        "samples": len(win),
        "samples_total": len(samples),
    }


def _host_load1():
    return os.getloadavg()[0]


def _curl(container, port, api_base, path):
    return dh.curl_status_body(container, f"http://localhost:{port}{api_base}{path}")


def ready_in(container, port, api_base, timeout_s=180):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        try:
            code, _ = _curl(container, port, api_base, "/health")
            if code == 200:
                return True
        except Exception:
            pass
        time.sleep(2)
    return False


def count_in(container, port, api_base):
    """Return the total entity count across all collection endpoints.

    Returns None if ALL three endpoint probes fail (dead/zero gateway),
    so callers can distinguish an unreachable gateway from a gateway that
    genuinely has zero entities.
    """
    total = 0
    all_failed = True
    for ep in ("/components", "/apps", "/areas"):
        try:
            _, body = _curl(container, port, api_base, ep)
            total += gateway_client.parse_items_count(body)
            all_failed = False
        except Exception:
            pass
    if all_failed:
        return None
    return total


def _warmup_gate(read, pid, container, port, api_base, timeout_s=180):
    """Block until the gateway USS and entity count are stable.

    Returns True when convergence is reached within the timeout, False
    when the timeout expires without convergence.

    Guards against a dead gateway: when count_in returns None for all
    probes, a sentinel value (None) is appended to the history so that
    the all-None window never satisfies entity_count_stable (which
    requires the stable value to be > 0).
    """
    warm, hist = [], []
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        warm.append(sampler.sample_once(read, pid, _host_load1()))
        count = count_in(container, port, api_base)
        # Append None when all probes fail; entity_count_stable rejects None.
        hist.append(count)
        if (len(warm) >= 5 and warmup.uss_derivative_below(warm[-5:], 5.0)
                and warmup.entity_count_stable(hist, 3)):
            return True
        time.sleep(2)
    return False


def _is_steady(out):
    """Check the steady-window USS slope CI and flag rising memory."""
    lo = out.get("uss_slope_lo95")
    hi = out.get("uss_slope_hi95")
    slope = out.get("uss_slope_b_s", 0.0)
    if lo is None or hi is None:
        return True, None
    # CI excludes zero AND magnitude exceeds threshold -> rising, not settled
    if lo > 0 and abs(slope) > _STEADY_SLOPE_THRESHOLD_B_S:
        return False, f"uss still rising {slope:.0f} B/s"
    return True, None


def _start_load_in_container(container, level, duration, port, api_base):
    """Start the in-container load generator as a detached background process.

    The load_gen module lives at /ws/benchmark/scaler/load_gen.py inside the
    synthetic benchmark image.  The demo image may not have /ws/benchmark; in
    that case this function returns None (no latency stats available).

    The generator is given ``duration + 5`` seconds so it outlasts the sample
    window and the caller can read the output file before teardown.

    Returns the path of the JSON output file written inside the container, or
    None when the module is absent.
    """
    check = dh.run([
        "docker", "exec", container, "sh", "-c",
        "test -f /ws/benchmark/scaler/load_gen.py && echo ok || echo missing",
    ])
    if "ok" not in check:
        return None
    cmd = (
        f"python3 /ws/benchmark/scaler/load_gen.py"
        f" --level {level}"
        f" --duration {int(duration + 5)}"
        f" --base-url http://localhost:{port}{api_base}"
        f" --out {_LOAD_STATS_PATH} &"
    )
    dh.run(["docker", "exec", "-d", container, "sh", "-c", cmd])
    return _LOAD_STATS_PATH


def _read_load_stats(container, out_path, wait_s=15):
    """Read load_stats JSON from the container. The load generator writes the file
    at the END of its run, so poll for it (up to wait_s) instead of reading once
    immediately after sampling (which races the write). Return dict or {}."""
    if out_path is None:
        return {}
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        try:
            data = json.loads(dh.run(["docker", "exec", container, "cat", out_path]))
            if data:
                return data
        except Exception:
            pass
        time.sleep(1)
    return {}


def run_cell(compose_file, project, service, gateway_proc, api_base, port,
             env, duration, interval, csv_path,
             load_level=None, load_duration=None):
    dh.compose_down(compose_file, project)  # clear any prior state of this project
    dh.compose_up(compose_file, project, env=env, service=service)
    try:
        container = dh.service_container(compose_file, project, service)
        if not ready_in(container, port, api_base):
            raise RuntimeError("gateway never became ready")
        pid = dh.resolve_gateway_pid(container, gateway_proc)
        clk = dh.getconf_clk_tck(container)
        read = lambda p, f: dh.read_proc(container, p, f)  # noqa: E731
        converged = _warmup_gate(read, pid, container, port, api_base)
        # Start the in-container load generator BEFORE the sample window so
        # the gateway is under load throughout the measurement period.
        load_out_path = None
        if load_level and load_level != "off":
            ld = load_duration if load_duration is not None else duration
            load_out_path = _start_load_in_container(container, load_level, ld, port, api_base)
        samples = sampler.sample_window(read, pid, clk, _host_load1,
                                        duration, interval, csv_path)
        out = summarize_window(samples, clk)
        # Read load stats while the container is still up (before finally teardown).
        out["load_stats"] = _read_load_stats(container, load_out_path)
        out["warmup_converged"] = converged
        # Default to steady; the flatness check may override this.
        out["steady"] = converged
        if converged:
            is_flat, reason = _is_steady(out)
            if not is_flat:
                out["steady"] = False
                out["steady_reason"] = reason
        else:
            out["steady_reason"] = "warmup did not converge"
        out["entity_count"] = count_in(container, port, api_base)
        out["status"] = "ok"
        out["container"] = container
        # Capture per-cell reproducibility metadata while the container is alive.
        # Each field degrades to "unknown" on failure so a capture error never
        # aborts the cell.
        meta = {}
        try:
            meta["image_digest"] = dh.image_digest_of_container(container)
        except Exception:
            meta["image_digest"] = "unknown"
        try:
            meta["gateway_sha"] = dh.read_gateway_sha(container)
        except Exception:
            meta["gateway_sha"] = "unknown"
        try:
            meta["allocator"] = dh.read_allocator_from_maps(container, pid)
        except Exception:
            meta["allocator"] = "unknown"
        try:
            meta["threads"] = dh.thread_census(container, pid)
        except Exception:
            meta["threads"] = {"total": 0}
        out["_meta"] = meta
        return out
    finally:
        dh.compose_down(compose_file, project)
