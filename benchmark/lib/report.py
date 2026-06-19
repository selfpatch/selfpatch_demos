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
"""Aggregate repeat cells and compute per-lane verdicts."""
from __future__ import annotations

from benchmark.lib.metrics import median, iqr, loglog_exponent

PASSTHROUGH = {"label", "samples", "samples_total", "entity_count", "node_count"}


def aggregate_cell(repeats):
    out = {"repeats": len(repeats)}
    for key in (repeats[0].keys() if repeats else []):
        vals = [r[key] for r in repeats]
        if key in PASSTHROUGH:
            out[key] = vals[0]            # carry verbatim (median across repeats noisy/meaningless)
            continue
        # Boolean fields: carry via majority vote (or first value for passthrough-like fields)
        if all(isinstance(v, bool) for v in vals):
            out[key] = sum(vals) >= len(vals) / 2
            continue
        # String fields (e.g. steady_reason): carry first non-None value
        if any(isinstance(v, str) for v in vals):
            out[key] = next((v for v in vals if v is not None), None)
            continue
        out[f"{key}_median"] = median(vals)
        out[f"{key}_q1"], out[f"{key}_q3"] = iqr(vals)
    return out


def scaling_verdict(entity_counts, uss_medians, uss_iqrs=None):
    """Return (verdict_str, meta) based on the CI of the log-log exponent.

    Verdict logic (B1):
      - ci_lo > 1.0  -> super-linear CONFIRMED
      - ci_hi < 1.0  -> sub-linear CONFIRMED
      - else         -> INDETERMINATE

    ``meta`` includes exponent, r2, ci_lo, ci_hi, and point_count.
    With <= 4 points, "indicative, few points" is appended.
    """
    exp, r2, ci_lo, ci_hi = loglog_exponent(entity_counts, uss_medians)
    n = len([x for x, y in zip(entity_counts, uss_medians) if x > 0 and y > 0])
    meta = {"exponent": exp, "r2": r2, "ci_lo": ci_lo, "ci_hi": ci_hi,
            "point_count": n}
    few = ", indicative, few points" if n <= 4 else ""
    ci_str = f"CI [{ci_lo:.2f},{ci_hi:.2f}]"
    if ci_lo > 1.0:
        verdict = (f"USS ~ entities^{exp:.2f} (R2={r2:.2f}) {ci_str} -> "
                   f"super-linear (swells): CONFIRMED{few}")
    elif ci_hi < 1.0:
        verdict = (f"USS ~ entities^{exp:.2f} (R2={r2:.2f}) {ci_str} -> "
                   f"sub-linear: CONFIRMED{few}")
    else:
        verdict = (f"USS ~ entities^{exp:.2f} (R2={r2:.2f}) {ci_str} -> "
                   f"INDETERMINATE (exponent CI spans 1; need more points/repeats){few}")
    return verdict, meta


def leak_verdict(uss_slope_ci, heap_growth_bytes, warmup_converged=True, has_sites=True):
    """Return a verdict string describing whether a leak is suspected.

    B4 honesty: "SUSPECTED" is only emitted when:
      - the corrected slope CI excludes zero (lo > 0), AND
      - the run is post-warmup (warmup_converged=True), AND
      - heaptrack attributes the growth to call sites (has_sites=True).
    Otherwise a more honest "inconclusive" or "no leak" verdict is returned.
    """
    _, lo, hi = uss_slope_ci
    ci_excludes_zero = lo > 0
    if ci_excludes_zero and warmup_converged and has_sites:
        return (f"USS slope CI [{lo:.1f},{hi:.1f}] B/s excludes zero; heap grew "
                f"{heap_growth_bytes / 1024 / 1024:.1f} MiB -> leak: SUSPECTED")
    if ci_excludes_zero and not warmup_converged:
        return (f"USS slope CI [{lo:.1f},{hi:.1f}] B/s excludes zero, but run was "
                f"short/unsettled -> inconclusive: short/unsettled run, "
                f"rerun --duration 1800")
    if ci_excludes_zero and not has_sites:
        return (f"USS slope CI [{lo:.1f},{hi:.1f}] B/s excludes zero, but no "
                f"attributable call sites -> inconclusive: warmup/cache fill, "
                f"rerun --duration 1800 for a definitive test")
    return f"USS slope CI [{lo:.1f},{hi:.1f}] B/s straddles zero -> NO LEAK detected"


def _mib(kib):
    return f"{kib / 1024:.1f} MiB"


def _steady_str(c):
    """Render the steady flag with the steady/total repeat count when available."""
    base = "yes" if c.get("steady", True) else "no"
    cnt = c.get("steady_count")
    return f"{base} ({cnt})" if cnt else base


def _cell_label_with_flags(c):
    """Return the cell label with markers for low-repeat or not-steady cells."""
    label = c["label"]
    flags = []
    if c.get("repeats", 3) < 3:
        flags.append("*")
    if not c.get("steady", True):
        flags.append("!")
    return label + "".join(flags)


def _high_load_warning(meta):
    """Return a bold warning line when the run was collected under high host load, else None."""
    if meta.get("high_host_load"):
        load1 = meta.get("host_load1", "?")
        nproc = meta.get("nproc", "?")
        return (f"**WARNING: collected under high host load (load1={load1} > {nproc} cores); "
                f"footprint/CPU not comparable to a clean run.**")
    return None


def render_footprint_markdown(cells, meta):
    lines = ["# Footprint", ""]
    warn = _high_load_warning(meta)
    if warn:
        lines += [warn, ""]
    lines += [
        f"_host: {meta.get('cpu_model')} x{meta.get('nproc')}, "
        f"{meta.get('mem_total_kb', 0) // 1024} MiB, allocator={meta.get('allocator')}_", ""]
    thread_info = meta.get("threads")
    if thread_info:
        lines += [f"_thread breakdown: {_thread_breakdown_str(thread_info)}_", ""]
    lines += [
        "| Config | steady | USS med | USS IQR (x-rep) | USS IQR (within) | PSS | "
        "RSS (upper) | CPU-cores | USS slope B/s [CI] | peak USS | threads | reps | samples |",
        "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for c in cells:
        steady_str = _steady_str(c)
        lo95 = c.get("uss_slope_lo95_median", c.get("uss_slope_lo95", 0))
        hi95 = c.get("uss_slope_hi95_median", c.get("uss_slope_hi95", 0))
        slope_val = c.get("uss_slope_b_s_median", c.get("uss_slope_b_s", 0))
        # Append "(rising)" when the CI excludes zero
        rising = " (rising)" if lo95 > 0 else ""
        lines.append(
            f"| {_cell_label_with_flags(c)} | {steady_str} | {_mib(c['uss_kib_median'])} | "
            f"{_mib(c['uss_kib_q1'])}-{_mib(c['uss_kib_q3'])} | "
            f"{_mib(c['uss_within_iqr_lo_median'])}-{_mib(c['uss_within_iqr_hi_median'])} | "
            f"{_mib(c['pss_kib_median'])} | {_mib(c['rss_kib_median'])} | "
            f"{c['cpu_cores_median']:.2f} | {slope_val:.0f} "
            f"[{lo95:.0f},{hi95:.0f}]{rising} | "
            f"{_mib(c['peak_uss_kib_median'])} | {c['num_threads_median']:.0f} | "
            f"{c['repeats']} | {c['samples']} |")
    lines.append("")
    lines.append("_\\* = fewer than 3 repeats, ! = steady check failed_")
    return "\n".join(lines) + "\n"


def render_footprint_chart(cells, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n_repeats = cells[0].get("repeats", "?") if cells else "?"
    labels = [_cell_label_with_flags(c) for c in cells]
    uss = [c["uss_kib_median"] / 1024 for c in cells]
    err = [[(c["uss_kib_median"] - c["uss_kib_q1"]) / 1024 for c in cells],
           [(c["uss_kib_q3"] - c["uss_kib_median"]) / 1024 for c in cells]]
    # Hatch pattern for unreliable cells (< 3 repeats or not steady)
    hatches = ["//" if (c.get("repeats", 3) < 3 or not c.get("steady", True))
               else "" for c in cells]
    fig, ax = plt.subplots()
    bars = ax.bar(labels, uss, yerr=err, capsize=4)
    for bar, hatch in zip(bars, hatches):
        bar.set_hatch(hatch)
    ax.set_ylabel("USS (MiB)")
    ax.set_title(f"Gateway footprint (USS, median +/- cross-repeat IQR, n={n_repeats} repeats per point)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def render_scaling_markdown(rows, verdict, fit, meta=None):
    lines = ["# Scaling", ""]
    if meta:
        warn = _high_load_warning(meta)
        if warn:
            lines += [warn, ""]
    lines += [f"**Verdict:** {verdict}", "",
              f"_fit: USS ~ entities^{fit['exponent']:.2f}, R2={fit['r2']:.2f}, "
              f"CI [{fit.get('ci_lo', 0):.2f},{fit.get('ci_hi', 0):.2f}]_", "",
              "| Discovered entities | nodes req | refresh ms | USS med | USS/entity | "
              "CPU-cores | reps | steady |", "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        steady_str = _steady_str(r)
        lines.append(
            f"| {r['entity_count']} | {r.get('node_count', '-')} | {r.get('refresh_ms', '-')} | "
            f"{_mib(r['uss_kib_median'])} | {r['uss_per_entity'] / 1024:.3f} MiB | "
            f"{r['cpu_cores_median']:.2f} | {r['repeats']} | {steady_str} |")
    return "\n".join(lines) + "\n"


def render_sweep_markdown(cells, meta=None):
    base = next((c for c in cells if c["label"] == "default"), None)
    lines = ["# Config sweep (one knob off default)", ""]
    if meta:
        warn = _high_load_warning(meta)
        if warn:
            lines += [warn, ""]
    lines += ["| Config | steady | USS med | USS IQR | CPU-cores | dUSS vs default |",
              "|---|---|---|---|---|---|"]
    for c in cells:
        d = (c["uss_kib_median"] - base["uss_kib_median"]) / 1024 if base else 0
        note = " (graph-perturbing)" if c["label"] == "plugins_minimal" else ""
        steady_str = _steady_str(c)
        lines.append(f"| {_cell_label_with_flags(c)}{note} | {steady_str} | "
                     f"{_mib(c['uss_kib_median'])} | "
                     f"{_mib(c['uss_kib_q1'])}-{_mib(c['uss_kib_q3'])} | "
                     f"{c['cpu_cores_median']:.2f} | {d:+.1f} MiB |")
    return "\n".join(lines) + "\n"


def render_sweep_bar(cells, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n_repeats = cells[0].get("repeats", "?") if cells else "?"
    labels = [_cell_label_with_flags(c) for c in cells]
    uss = [c["uss_kib_median"] / 1024 for c in cells]
    err = [[(c["uss_kib_median"] - c["uss_kib_q1"]) / 1024 for c in cells],
           [(c["uss_kib_q3"] - c["uss_kib_median"]) / 1024 for c in cells]]
    colors = ["tab:orange" if c["label"] == "default" else "tab:blue" for c in cells]
    hatches = ["//" if (c.get("repeats", 3) < 3 or not c.get("steady", True))
               else "" for c in cells]
    fig, ax = plt.subplots()
    bars = ax.bar(labels, uss, yerr=err, capsize=4, color=colors)
    for bar, hatch in zip(bars, hatches):
        bar.set_hatch(hatch)
    ax.set_ylabel("USS (MiB)")
    ax.set_title(f"Footprint by config (default highlighted, n={n_repeats} repeats per point)")
    plt.setp(ax.get_xticklabels(), rotation=30, ha="right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def render_heap_markdown(summary, slope_ci, verdict, meta=None):
    lines = ["# Heap / leak", ""]
    if meta:
        warn = _high_load_warning(meta)
        if warn:
            lines += [warn, ""]
    lines += [
        "_Note: the USS slope is measured from /proc while heaptrack is active. "
        "Heaptrack's own shadow memory (allocation tracking metadata) lives in the "
        "gateway address space and inflates /proc USS.  A SUSPECTED verdict or a "
        "rising slope is not solely attributable to the gateway itself._", "",
        f"**Verdict:** {verdict}", "",
        f"- total leaked at exit: {summary.total_leaked_bytes / 1024 / 1024:.1f} MiB",
        f"- peak heap: {summary.peak_heap_bytes / 1024 / 1024:.1f} MiB",
        f"- USS slope 95% CI: [{slope_ci[1]:.1f}, {slope_ci[2]:.1f}] B/s", "",
        "Top allocation call-sites:", ""]
    if summary.top_sites:
        for b, site in summary.top_sites:
            lines.append(f"- {b / 1024 / 1024:.2f} MiB :: {site}")
    else:
        lines.append("_none attributed: no per-call-site leaks in heaptrack output. "
                     "On a short clean-shutdown run, USS growth is warmup/cache fill, "
                     "not a leak - run longer (e.g. --duration 1800) for a definitive test._")
    return "\n".join(lines) + "\n"


def render_memcheck_markdown(s):
    return ("# Valgrind memcheck (short controlled run)\n\n"
            f"- definitely lost: {s.definitely_lost_bytes} bytes in "
            f"{s.definitely_lost_blocks} blocks\n"
            f"- indirectly lost: {s.indirectly_lost_bytes} bytes\n\n"
            "_still-reachable excluded (FastDDS shutdown noise)._\n")


def _thread_breakdown_str(threads: dict) -> str:
    """Format thread census dict as a compact breakdown string."""
    if not threads or threads.get("total", 0) == 0:
        return "unknown"
    total = threads.get("total", 0)
    # Show every non-empty category (the census categories are not a fixed set),
    # largest first, so no group is silently dropped.
    cats = sorted(((k, v) for k, v in threads.items() if k != "total" and v),
                  key=lambda kv: -kv[1])
    parts = [f"{k}={v}" for k, v in cats]
    return f"total={total} ({', '.join(parts)})" if parts else f"total={total}"


def render_load_markdown(cells: list) -> str:
    """Render a load-lane report table.

    Each cell is a dict with keys:
        level, uss_kib_median, cpu_cores_median, num_threads_median,
        p50_ms, p95_ms, threads (optional census dict), repeats, samples.

    Appends an optimization-signal line based on the data.
    """
    lines = ["# Load lane", "",
             "| Load level | USS med | CPU-cores | threads | p50 ms | p95 ms | reps |",
             "|---|---|---|---|---|---|---|"]
    for c in cells:
        threads_str = _thread_breakdown_str(c.get("threads", {}))
        lines.append(
            f"| {c.get('level', '?')} | {_mib(c.get('uss_kib_median', 0))} | "
            f"{c.get('cpu_cores_median', 0.0):.2f} | {threads_str} | "
            f"{c.get('p50_ms', 0.0):.1f} | {c.get('p95_ms', 0.0):.1f} | "
            f"{c.get('repeats', 1)} |"
        )
    lines.append("")

    # Optimization signal
    signals = []
    off_cell = next((c for c in cells if c.get("level") == "off"), None)
    heavy_cell = next((c for c in cells if c.get("level") == "heavy"), None)
    if off_cell and heavy_cell:
        uss_delta = (heavy_cell.get("uss_kib_median", 0)
                     - off_cell.get("uss_kib_median", 0)) / 1024
        cpu_heavy = heavy_cell.get("cpu_cores_median", 0.0)
        p95_heavy = heavy_cell.get("p95_ms", 0.0)
        thread_total = heavy_cell.get("threads", {}).get("total", 0)
        if uss_delta > 10:
            signals.append(f"USS +{uss_delta:.0f} MiB under heavy load -> "
                           "request buffers not freed between polls")
        if cpu_heavy > 0.8:
            signals.append(f"CPU {cpu_heavy:.2f} cores at heavy load -> "
                           "consider bounded HTTP thread pool")
        if p95_heavy > 200:
            signals.append(f"p95 latency {p95_heavy:.0f} ms -> "
                           "HTTP thread pool likely saturated")
        if thread_total > 40:
            signals.append(f"{thread_total} threads under heavy load -> "
                           "cap executor thread count")
    if signals:
        lines.append("**Optimization signals:**")
        for sig in signals:
            lines.append(f"- {sig}")
    else:
        lines.append("_No optimization signals triggered at these load levels._")

    return "\n".join(lines) + "\n"


def render_fault_markdown(rows: list, meta: dict = None) -> str:
    """Render the fault-lane report table + optimization signal.

    Each row dict has keys:
        n, mode, peak_uss_delta_mib, peak_cpu_cores, capture_duration_s,
        residual_mib, recovered, rosbag_got, rosbag_total.

    The table is followed by a one-line optimization signal derived from the
    data (e.g. capture duration growth rate and rosbag contention ratio).
    """
    lines = ["# Fault / snapshot lane", ""]
    if meta:
        warn = _high_load_warning(meta)
        if warn:
            lines += [warn, ""]
        lines += [
            f"_host: {meta.get('cpu_model')} x{meta.get('nproc')}, "
            f"{meta.get('mem_total_kb', 0) // 1024} MiB_", ""]
    lines += [
        "_Each (N, mode) cell is a single-sample measurement (n=1): one fresh "
        "container per N, one burst. Values are not IQR-qualified; treat them as "
        "point estimates only._", "",
        "| N faults | mode | peak USS delta | peak CPU | capture duration s | "
        "residual | recovered | rosbag got/N |",
        "|---|---|---|---|---|---|---|---|",
    ]
    for r in rows:
        rosbag_str = (
            f"{r['rosbag_got']}/{r['rosbag_total']}"
            if r["rosbag_total"] > 0
            else "n/a"
        )
        lines.append(
            f"| {r['n']} | {r['mode']} | "
            f"{r['peak_uss_delta_mib']:.2f} MiB | "
            f"{r['peak_cpu_cores']:.2f} | "
            f"{r['capture_duration_s']:.2f} | "
            f"{r['residual_mib']:.2f} MiB | "
            f"{'yes' if r['recovered'] else 'no'} | "
            f"{rosbag_str} |"
        )
    lines.append("")

    # Optimization signal: derive from the data rows.
    signals = _fault_optimization_signals(rows)
    if signals:
        lines.append("**Optimization signals:**")
        for sig in signals:
            lines.append(f"- {sig}")
    else:
        lines.append("_No optimization signals triggered._")

    return "\n".join(lines) + "\n"


def _fault_optimization_signals(rows: list) -> list:
    """Derive optimization signals from fault-lane rows.

    Checks:
    - Capture duration grows super-linearly with N (data mode).
    - Rosbag contention: fraction of faults getting rosbag is < 50% at N>1.
    - Peak USS delta grows with N (more faults -> more snapshot buffers).
    """
    signals = []

    # Collect data-mode rows sorted by N for growth analysis.
    data_rows = sorted(
        [r for r in rows if r["mode"] == "data" and r["n"] > 0],
        key=lambda r: r["n"],
    )
    if len(data_rows) >= 2:
        n_lo, n_hi = data_rows[0]["n"], data_rows[-1]["n"]
        dur_lo = data_rows[0]["capture_duration_s"]
        dur_hi = data_rows[-1]["capture_duration_s"]
        if n_lo > 0 and dur_lo > 0 and n_hi > n_lo:
            dur_ratio = dur_hi / dur_lo
            n_ratio = n_hi / n_lo
            if dur_ratio > n_ratio * 0.9:
                signals.append(
                    f"capture duration grows ~O(N): {dur_lo:.1f}s at N={n_lo} -> "
                    f"{dur_hi:.1f}s at N={n_hi} -> async/queue snapshot capture"
                )
        uss_lo = data_rows[0]["peak_uss_delta_mib"]
        uss_hi = data_rows[-1]["peak_uss_delta_mib"]
        if uss_lo > 0 and uss_hi > uss_lo * 1.5:
            signals.append(
                f"peak USS delta +{uss_hi:.1f} MiB at N={n_hi} vs "
                f"+{uss_lo:.1f} MiB at N={n_lo} -> snapshot buffers not bounded"
            )

    # Rosbag contention: at N>1, how many got rosbag?
    rosbag_rows = sorted(
        [r for r in rows if r["mode"] == "rosbag" and r["rosbag_total"] > 1],
        key=lambda r: r["n"],
    )
    for r in rosbag_rows:
        frac = r["rosbag_got"] / r["rosbag_total"] if r["rosbag_total"] > 0 else 0
        if frac < 0.5:
            signals.append(
                f"rosbag contention: {r['rosbag_got']}/{r['rosbag_total']} faults "
                f"got rosbag at N={r['n']} -> rosbag writer is shared, "
                f"consider per-fault capture queue"
            )
            break  # one contention signal is enough

    return signals


def render_fault_chart(rows: list, out_png: str) -> None:
    """Plot peak USS delta and capture duration vs N for data vs +rosbag series."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    modes = sorted({r["mode"] for r in rows})
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4))

    for mode in modes:
        series = sorted([r for r in rows if r["mode"] == mode], key=lambda r: r["n"])
        xs = [r["n"] for r in series]
        uss = [r["peak_uss_delta_mib"] for r in series]
        dur = [r["capture_duration_s"] for r in series]
        label = f"+rosbag" if mode == "rosbag" else "data only"
        ax1.plot(xs, uss, marker="o", label=label)
        ax2.plot(xs, dur, marker="o", label=label)

    ax1.set_xlabel("N concurrent faults")
    ax1.set_ylabel("Peak USS delta (MiB)")
    ax1.set_title("Peak USS delta vs N")
    ax1.legend()

    ax2.set_xlabel("N concurrent faults")
    ax2.set_ylabel("Capture duration (s)")
    ax2.set_title("Capture duration vs N")
    ax2.legend()

    fig.suptitle("Fault/snapshot burst impact")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def validate_synthetic(demo_uss_per_entity, synth_rows, demo_entity_count):
    """Compare the demo USS-per-entity to the synthetic row whose entity_count is closest.

    ``synth_rows`` is a list of dicts with keys ``entity_count`` and ``uss_per_entity``.
    ``demo_entity_count`` is the entity count discovered on the demo run.

    Picks the synthetic row whose ``entity_count`` is closest to ``demo_entity_count``.
    If the closest match is more than 50% away from the demo entity count, reports
    the gap instead of a potentially misleading ratio.
    """
    if demo_uss_per_entity <= 0:
        return "validation: INCONCLUSIVE (demo USS per entity is zero)"
    if not synth_rows:
        return "validation: INCONCLUSIVE (no synthetic rows)"
    best = min(synth_rows, key=lambda r: abs(r["entity_count"] - demo_entity_count))
    synth_ec = best["entity_count"]
    synth_upe = best["uss_per_entity"]
    distance_frac = abs(synth_ec - demo_entity_count) / max(1, demo_entity_count)
    if distance_frac > 0.5:
        return (f"validation: no synthetic point within 50% of demo entity count "
                f"(demo={demo_entity_count}, closest synth={synth_ec}); "
                f"run scaling with a closer --entities value for a valid comparison")
    ratio = synth_upe / demo_uss_per_entity
    if 0.7 <= ratio <= 1.4:
        return (f"synthetic/demo USS-per-entity ratio {ratio:.2f} -> consistent "
                f"(synth N={synth_ec}, demo N={demo_entity_count})")
    return (f"synthetic/demo USS-per-entity ratio {ratio:.2f} -> DIVERGES "
            f"(synth N={synth_ec}, demo N={demo_entity_count}; "
            f"synthetic graph does not reproduce demo per-entity cost)")


def render_scaling_chart(groups_with_fits, out_png):
    """Plot scaling curves; each series uses its own fit.

    ``groups_with_fits`` is a list of ``(rows, fit)`` pairs, one per refresh-ms group.
    Each ``fit`` dict must have keys ``exponent``, ``r2``, and ``refresh_ms``.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    all_rows = [r for rows, _fit in groups_with_fits for r in rows]
    n_repeats = all_rows[0].get("repeats", "?") if all_rows else "?"
    fig, ax = plt.subplots()
    unreliable_plotted = False
    for rows, fit in groups_with_fits:
        refresh = fit.get("refresh_ms", "?")
        xs = [r["entity_count"] for r in rows]
        ys = [r["uss_kib_median"] / 1024 for r in rows]
        lo = [(r["uss_kib_median"] - r["uss_kib_q1"]) / 1024 for r in rows]
        hi = [(r["uss_kib_q3"] - r["uss_kib_median"]) / 1024 for r in rows]
        ax.errorbar(xs, ys, yerr=[lo, hi], marker="o", capsize=4,
                    label=f"USS refresh={refresh}ms (med +/- IQR)")
        # Overlay red-X for unreliable points (< 3 repeats or not steady)
        for r, x, y in zip(rows, xs, ys):
            if r.get("repeats", 3) < 3 or not r.get("steady", True):
                label = "unreliable point" if not unreliable_plotted else None
                ax.plot(x, y, "rx", markersize=10, zorder=5, label=label)
                unreliable_plotted = True
        # Use this group's own fit for the dashed power-law line
        if len(xs) >= 2 and xs[0] > 0:
            x0, y0 = xs[0], ys[0]
            ax.plot(xs, [y0 * (x / x0) ** fit["exponent"] for x in xs], "--",
                    label=f"fit exp={fit['exponent']:.2f} R2={fit['r2']:.2f} "
                          f"(refresh={refresh}ms)")
    if all_rows:
        all_xs = sorted({r["entity_count"] for r in all_rows})
        first_row = min(all_rows, key=lambda r: r["entity_count"])
        x0 = first_row["entity_count"]
        y0 = first_row["uss_kib_median"] / 1024
        if x0 > 0:
            ax.plot(all_xs, [y0 * (x / x0) for x in all_xs], ":", label="linear reference")
    ax.set_xlabel("discovered entity count")
    ax.set_ylabel("USS (MiB)")
    ax.set_title(f"Gateway footprint vs project size (n={n_repeats} repeats per point)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)
