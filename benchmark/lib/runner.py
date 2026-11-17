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
from concurrent.futures import ThreadPoolExecutor

from benchmark.lib import docker_helpers as dh
from benchmark.lib import gateway_client, sampler, warmup
from benchmark.lib.metrics import median, iqr, peak_uss, slope_ci95, steady_window

_LOAD_STATS_PATH = "/tmp/load_stats.json"

# Steady-window flatness threshold: if the absolute USS slope exceeds this value
# (in B/s) AND the 95% CI excludes zero, the window is flagged not-steady.
_STEADY_SLOPE_THRESHOLD_B_S = 1024.0

# Upper bound on how long the warmup gate blocks (also the default in
# _warmup_gate).  The load generator for a load cell is sized to outlast warmup
# plus the sample window so it stays up while we warm under load.
_WARMUP_TIMEOUT_S = 180


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


# Per-process stats that are summed over gateway and fault manager.
_SUM_KEYS = ("uss_kib", "pss_kib", "rss_kib", "cpu_cores", "num_threads")


def merge_fault_manager(out, fm):
    """Add the fault manager's stats to a gateway cell as ``fm_*`` and ``sum_*``.

    ``fm`` is a summarize_window dict. A failed or empty window adds no keys
    except ``fm_status``, so it never reads as a zero-cost process.
    """
    if fm.get("samples", 0) == 0:
        out["fm_status"] = "failed"
        return
    out["fm_status"] = "ok"
    for k, v in fm.items():
        out[f"fm_{k}"] = v
    for k in _SUM_KEYS:
        out[f"sum_{k}"] = out[k] + fm[k]


# A process must be sampled over this share of the window, else it died or was lost.
_MIN_COVERAGE = 0.8


def covered(samples, duration):
    """True when ``samples`` span at least _MIN_COVERAGE of the ``duration`` window."""
    return len(samples) >= 2 and samples[-1].t - samples[0].t >= _MIN_COVERAGE * duration


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


def _warmup_verdict(read, pid, container, port, api_base, timeout_s=_WARMUP_TIMEOUT_S,
                    fm_pid=None):
    """Block until the gateway and the fault manager are stable, or the timeout.

    Returns {"gateway": bool, "fm": bool or None}: whether each process was stable
    at the end (``fm`` is None without a fault manager). The wait ends early only
    when both are stable at once, so a gateway verdict never depends on the fault
    manager settling.

    Guards against a dead gateway: when count_in returns None for all
    probes, a sentinel value (None) is appended to the history so that
    the all-None window never satisfies entity_count_stable (which
    requires the stable value to be > 0).
    """
    warm, fm_warm, hist = [], [], []
    gw_ok = False
    fm_ok = None if fm_pid is None else False
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        warm.append(sampler.sample_once(read, pid, _host_load1()))
        if fm_pid is not None:
            fm_warm.append(sampler.sample_once(read, fm_pid, _host_load1()))
        count = count_in(container, port, api_base)
        # Append None when all probes fail; entity_count_stable rejects None.
        hist.append(count)
        gw_ok = (len(warm) >= 5 and warmup.uss_derivative_below(warm[-5:], 5.0)
                 and warmup.entity_count_stable(hist, 3))
        if fm_pid is not None:
            fm_ok = (len(fm_warm) >= 5 and warmup.uss_derivative_below(fm_warm[-5:], 5.0))
        if gw_ok and fm_ok is not False:
            break
        time.sleep(2)
    return {"gateway": bool(gw_ok), "fm": fm_ok}


def _warmup_gate(read, pid, container, port, api_base, timeout_s=_WARMUP_TIMEOUT_S,
                 fm_pid=None):
    """True when every sampled process was stable before the timeout."""
    v = _warmup_verdict(read, pid, container, port, api_base, timeout_s, fm_pid)
    return v["gateway"] and v["fm"] is not False


def _is_steady(out, prefix=""):
    """Check the steady-window USS slope CI and flag rising memory.

    ``prefix`` selects another process's keys (``fm_`` for the fault manager).
    """
    lo = out.get(f"{prefix}uss_slope_lo95")
    hi = out.get(f"{prefix}uss_slope_hi95")
    slope = out.get(f"{prefix}uss_slope_b_s", 0.0)
    if lo is None or hi is None:
        return True, None
    # CI excludes zero AND magnitude exceeds threshold -> rising, not settled
    if lo > 0 and abs(slope) > _STEADY_SLOPE_THRESHOLD_B_S:
        return False, f"uss still rising {slope:.0f} B/s"
    return True, None


def _start_load_in_container(container, level, duration, port, api_base, cpus=None,
                             progress=None):
    """Start the in-container load generator as a detached background process.

    The load_gen module lives at /ws/benchmark/scaler/load_gen.py inside the
    synthetic benchmark image.  The demo image may not have /ws/benchmark; in
    that case this function returns None (no latency stats available).

    The generator is given ``duration + 5`` seconds so it outlasts the sample
    window and the caller can read the output file before teardown.

    ``progress`` names a file inside the container that the running counts go to.

    Returns the path of the JSON output file written inside the container, or
    None when the module is absent.
    """
    check = dh.run([
        "docker", "exec", container, "sh", "-c",
        "test -f /ws/benchmark/scaler/load_gen.py && echo ok || echo missing",
    ])
    if "ok" not in check:
        return None
    pin = f"taskset -c {cpus} " if cpus else ""
    cmd = (
        f"{pin}python3 /ws/benchmark/scaler/load_gen.py"
        f" --level {level}"
        f" --duration {int(duration + 5)}"
        f" --base-url http://localhost:{port}{api_base}"
        f" --out {_LOAD_STATS_PATH}"
        + (f" --progress {progress}" if progress else "") + " &"
    )
    dh.run(["docker", "exec", "-d", container, "sh", "-c", cmd])
    return _LOAD_STATS_PATH


def _stop_load_in_container(container):
    """Signal the in-container load generator to stop and flush its stats.

    The generator handles SIGTERM by ending its run early and writing the stats
    file, so stopping it right after the sample window means the stats are ready
    to read before teardown even though it was started (before warmup) with a
    runtime that outlasts the window.  Best-effort: a missing process is fine.
    """
    try:
        dh.run(["docker", "exec", container, "pkill", "-TERM", "-f",
                "load_gen.py"])
    except Exception:
        pass


def _read_load_stats(container, out_path, wait_s=30):
    """Read load_stats JSON from the container. The load generator writes the file
    at the END of its run, so poll for it (up to wait_s) instead of reading once
    immediately after sampling (which races the write). The budget allows for the
    generator's own teardown after SIGTERM (heavy level joins up to 4 SSE workers
    at 5s each before writing stats). Return dict or {}."""
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


def _fm_expected(env):
    """True when the setup starts a fault manager (BENCH_FAULT_MANAGER not 0, no tool)."""
    return (env or {}).get("BENCH_FAULT_MANAGER", "1") != "0" and \
        (env or {}).get("TOOL", "none") == "none"


def _resolve_fault_manager(container):
    """PID of the fault manager in the container, or None when it does not run."""
    try:
        return dh.resolve_pid_by_exe(container, dh.FAULT_MANAGER_EXE)
    except RuntimeError:
        return None


def _process_meta(container, gw_pid, fm_pid):
    """Resolved PID and command line of each sampled process, for the run metadata."""
    procs = {}
    for name, pid in (("gateway_node", gw_pid), (dh.FAULT_MANAGER_EXE, fm_pid)):
        if pid is None:
            continue
        try:
            procs[name] = {"pid": pid, "cmdline": dh.read_cmdline(container, pid)}
        except Exception:
            procs[name] = {"pid": pid, "cmdline": "unknown"}
    return procs


def _sample_both(read, pid, fm_pid, clk, duration, interval, csv_path):
    """Sample the gateway and, when present, the fault manager over the same window.

    Returns (gateway_samples, fault_manager_samples). The fault manager's rows go
    to ``<csv stem>.fm.csv`` next to the gateway's.
    """
    if fm_pid is None:
        return (sampler.sample_window(read, pid, clk, _host_load1, duration,
                                      interval, csv_path), [])
    fm_csv = str(csv_path).removesuffix(".csv") + ".fm.csv"
    with ThreadPoolExecutor(max_workers=2) as pool:
        gw = pool.submit(sampler.sample_window, read, pid, clk, _host_load1,
                         duration, interval, csv_path)
        fm = pool.submit(sampler.sample_window, read, fm_pid, clk, _host_load1,
                         duration, interval, fm_csv)
        return gw.result(), fm.result()


def run_cell(compose_file, project, service, gateway_proc, api_base, port,
             env, duration, interval, csv_path,
             load_level=None, load_duration=None, probe_fn=None):
    dh.compose_down(compose_file, project)  # clear any prior state of this project
    dh.compose_up(compose_file, project, env=env, service=service)
    try:
        container = dh.service_container(compose_file, project, service)
        if not ready_in(container, port, api_base):
            raise RuntimeError("gateway never became ready")
        pid = dh.resolve_gateway_pid(container, gateway_proc)
        fm_pid = _resolve_fault_manager(container)
        clk = dh.getconf_clk_tck(container)
        read = lambda p, f: dh.read_proc(container, p, f)  # noqa: E731
        # For a load cell, start the generator BEFORE the warmup gate so the
        # gateway warms and settles UNDER load - warming on the idle gateway
        # would let the loaded USS/CPU keep ramping into the steady window.  The
        # generator is sized to outlast warmup + window and is stopped explicitly
        # after the window so it flushes its stats in time.
        load_out_path = None
        if load_level and load_level != "off":
            ld = load_duration if load_duration is not None else duration
            gen_runtime = _WARMUP_TIMEOUT_S + ld + 30
            load_out_path = _start_load_in_container(
                container, load_level, gen_runtime, port, api_base)
        verdict = _warmup_verdict(read, pid, container, port, api_base, fm_pid=fm_pid)
        converged = verdict["gateway"]
        fm_converged = bool(verdict["fm"])
        samples, fm_samples = _sample_both(read, pid, fm_pid, clk, duration,
                                           interval, csv_path)
        out = summarize_window(samples, clk)
        gw_covered = covered(samples, duration)
        fm_covered = True
        if fm_pid is not None:
            merge_fault_manager(out, summarize_window(fm_samples, clk))
            fm_covered = covered(fm_samples, duration)
            if out.get("fm_status") == "ok" and not fm_covered:
                span = fm_samples[-1].t - fm_samples[0].t if fm_samples else 0.0
                out["fm_status"] = "incomplete"
                out["fm_status_reason"] = (
                    f"fault manager sampled for {span:.0f} s of a {duration:.0f} s window")
        elif _fm_expected(env):
            out["fm_status"] = "failed"
            out["fm_status_reason"] = "the fault manager is expected but was not found"
        else:
            out["fm_status"] = "absent"
        # Stop the generator (flushes stats), then read them while the container
        # is still up (before the finally teardown).
        if load_out_path:
            _stop_load_in_container(container)
        out["load_stats"] = _read_load_stats(container, load_out_path)
        out["warmup_converged"] = converged
        # A window that gathered no samples (e.g. the process exited and /proc
        # reads failed) is a FAILED measurement - summarize_window already flagged
        # it. Keep that: do not relabel it ok/steady, or an empty cell would
        # render downstream as a valid zero-slope steady measurement.
        # ``steady`` is the gateway's own verdict; the fault manager has
        # ``fm_steady``, so an unsteady fault manager never drops gateway data.
        no_samples = out.get("samples", 0) == 0
        out["steady"] = converged and not no_samples and gw_covered
        if no_samples:
            out["steady_reason"] = out.get("error", "no samples collected")
        elif not gw_covered:
            out["steady_reason"] = "gateway was not sampled over the whole window"
        elif converged:
            is_flat, reason = _is_steady(out)
            if not is_flat:
                out["steady"] = False
                out["steady_reason"] = reason
        else:
            out["steady_reason"] = "warmup did not converge"
        if out.get("fm_status") == "ok":
            out["fm_steady"] = fm_converged
            if fm_converged:
                fm_flat, fm_reason = _is_steady(out, "fm_")
                if not fm_flat:
                    out["fm_steady"] = False
                    out["fm_steady_reason"] = fm_reason
            else:
                out["fm_steady_reason"] = "warmup did not converge"
        elif fm_pid is not None:
            out["fm_steady"] = False
        out["entity_count"] = count_in(container, port, api_base)
        if probe_fn is not None:
            out["probe"] = probe_fn(container)
        if not no_samples:
            # A gateway sampled for less than the window is as incomplete as a fault manager is.
            out["status"] = "ok" if gw_covered else "incomplete"  # else keep "failed"
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
        meta["processes"] = _process_meta(container, pid, fm_pid)
        out["_meta"] = meta
        return out
    finally:
        dh.compose_down(compose_file, project)
