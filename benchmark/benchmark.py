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
"""Benchmark harness CLI for the ros2_medkit gateway."""
from __future__ import annotations
import argparse
import json
import os
import time
from pathlib import Path

import yaml

from benchmark import turtlebot3
from benchmark.lib import docker_helpers as dh
from benchmark.lib import report, runmeta, runner
from benchmark.lib import sampler
from benchmark.lib.config_sweep import apply_override_at
from benchmark.lib.fault_injector import build_fault_rows, inject_faults
from benchmark.lib.leak_parse import parse_heaptrack_summary, parse_memcheck_summary
from benchmark.lib.metrics import slope_ci95

REPO = Path(__file__).resolve().parents[1]
RESULTS = Path(__file__).resolve().parent / "results"
GATEWAY_PORT = 8080

SYN_COMPOSE = str(Path(__file__).resolve().parent / "profiles" / "synthetic.compose.yml")
# Fault lane uses a dedicated service in the same compose file.
_FAULT_SERVICE = "bench-fault"
_FAULT_PROC = "gateway_node"
_DEFAULT_FAULTS = "1,2,4,8,16"


def _run_dir(root, lane):
    d = (root or RESULTS / time.strftime("%Y%m%d-%H%M%S")) / lane
    d.mkdir(parents=True, exist_ok=True)
    return d


def _resolve_root(args, root):
    """Resolve the run-dir root for a lane.

    An explicit ``root`` (internal callers like cmd_all) wins; otherwise the
    global ``--run-dir`` is honored so several lane invocations write into ONE
    shared run dir (CI runs scaling and footprint as separate processes and then
    compares them together); otherwise None -> a fresh timestamped dir per lane.
    """
    if root is not None:
        return root
    rd = getattr(args, "run_dir", None)
    return (RESULTS / rd) if rd else None


def _strip(cell):
    # Drop non-numeric / non-aggregatable keys before aggregate_cell (which medians
    # every remaining key). load_stats is a dict; status/container/_meta are not data.
    return {k: v for k, v in cell.items()
            if k not in ("status", "container", "_meta", "load_stats")}


def _aggregate_steady(reps):
    # Aggregate steady-state stats over the repeats that actually reached steady
    # state; a still-ramping repeat is not a valid steady-state sample. Disclose
    # how many of the repeats were steady so the median is not silently inflated
    # by an unsettled run.
    steady = [r for r in reps if r.get("steady", True)]
    used = steady if steady else reps
    cell = report.aggregate_cell([_strip(r) for r in used])
    cell["steady"] = bool(reps) and len(steady) == len(reps)
    cell["steady_count"] = f"{len(steady)}/{len(reps)}"
    return cell


def _args_dict(args):
    """Return vars(args) with the 'func' callable entry removed."""
    return {k: v for k, v in vars(args).items() if k != "func"}


def _read_demo_refresh_ms(params_path):
    """Read refresh_interval_ms from a medkit params YAML under the namespaced key.

    The turtlebot3 params file uses:
        diagnostics.ros2_medkit_gateway.ros__parameters.refresh_interval_ms

    Falls back to "unknown" if the file is missing or the key is absent.
    """
    try:
        data = yaml.safe_load(Path(params_path).read_text())
        # Walk: diagnostics -> ros2_medkit_gateway -> ros__parameters
        node = data.get("diagnostics", {})
        node = node.get("ros2_medkit_gateway", {})
        node = node.get("ros__parameters", {})
        val = node.get("refresh_interval_ms")
        if val is not None:
            return int(val)
    except Exception:
        pass
    return "unknown"


def check_host_load(strict=False):
    """Check host load and warn (or abort) when it exceeds cpu_count.

    Returns a dict with keys ``host_load1`` and ``high_host_load``.
    When ``strict`` is True and load is high, raises RuntimeError so the
    caller can abort the lane before starting containers.
    """
    load1 = os.getloadavg()[0]
    nproc = os.cpu_count() or 1
    high = load1 > nproc
    if high:
        print(f"WARNING: host load1={load1:.2f} exceeds cpu_count={nproc}; "
              "footprint/CPU numbers under high load are not comparable across runs.")
        if strict:
            raise RuntimeError(
                f"--strict: aborting lane (load1={load1:.2f} > cpu_count={nproc})")
    return {"host_load1": load1, "high_host_load": high}


def _write_metadata(out_dir, args, cell_meta, refresh_ms):
    """Write run_metadata.json for any lane.

    ``args`` is the parsed argparse.Namespace for the lane.
    ``cell_meta`` is the ``_meta`` dict from the first successful cell (or {}).
    ``refresh_ms`` is the actual refresh interval used (int or "unknown").
    """
    load_info = check_host_load(strict=False)
    meta = runmeta.collect(
        args=_args_dict(args),
        demo_name=getattr(args, "demo", "unknown"),
        cell_meta=cell_meta,
        refresh_ms=refresh_ms,
    )
    # Overwrite host_load1 with the value captured at write time (end of lane),
    # and propagate the high_host_load flag.
    meta["host_load1"] = load_info["host_load1"]
    meta["high_host_load"] = load_info["high_host_load"]
    (out_dir / "run_metadata.json").write_text(json.dumps(meta, indent=2, default=str))
    return meta


def _load_label(level):
    """Return a cell label that includes the load level."""
    return "default" if level == "off" else f"default+load={level}"


def cmd_footprint(args, root=None):
    check_host_load(strict=getattr(args, "strict", False))
    load_level = getattr(args, "load", "off")
    demo = turtlebot3.TURTLEBOT3
    out = _run_dir(_resolve_root(args, root), "footprint")
    file = str(REPO / demo.compose_file)
    env = dict(os.environ, COMPOSE_PROFILES=demo.compose_profile)
    reps, status = [], "ok"
    for rep in range(args.repeats):
        try:
            cell = runner.run_cell(file, "bench_fp", demo.gateway_service,
                                   demo.gateway_proc, demo.api_base, GATEWAY_PORT, env,
                                   args.duration, args.interval, str(out / f"rep{rep}.csv"),
                                   load_level=load_level if load_level != "off" else None)
            reps.append(cell)
            (out / f"rep{rep}.json").write_text(json.dumps(cell, indent=2, default=str))
        except Exception as exc:
            status = "partial"
            (out / f"rep{rep}.error").write_text(str(exc))
    cell = _aggregate_steady(reps) if reps else {}
    cell["label"] = _load_label(load_level)
    # Use refresh_ms from the demo params file (the actual value, not hardcoded).
    refresh_ms = _read_demo_refresh_ms(REPO / demo.base_params_rel)
    first_meta = reps[0].get("_meta", {}) if reps else {}
    _write_metadata(out, args, first_meta, refresh_ms)
    (out / "summary.json").write_text(json.dumps({"status": status, "cell": cell},
                                                 indent=2, default=str))
    if reps:
        meta = json.loads((out / "run_metadata.json").read_text())
        # Surface thread breakdown in metadata for the footprint report header.
        if first_meta.get("threads"):
            meta["threads"] = first_meta["threads"]
        (out / "report.md").write_text(report.render_footprint_markdown([cell], meta))
        report.render_footprint_chart([cell], str(out / "footprint.png"))
    print(f"footprint: {status} -> {out}")


def scaling_rows(cells):
    rows = []
    for c in cells:
        r = dict(c)
        ec = c["entity_count"]
        r["uss_per_entity"] = c["uss_kib_median"] / ec if ec else 0
        rows.append(r)
    return rows


def _syn_cell(entities, args, out, tag):
    env = dict(os.environ, BENCH_ENTITIES=str(entities),
               BENCH_TOPICS_PER_NODE=str(args.topics_per_node),
               BENCH_MSG_TYPES=str(args.msg_types))
    cell = runner.run_cell(SYN_COMPOSE, f"bench_syn_{tag}", "bench", "gateway_node",
                           "/api/v1", GATEWAY_PORT, env, args.duration, args.interval,
                           str(out / f"{tag}.csv"))
    cell["node_count"] = entities
    return cell


def cmd_scaling(args, root=None):
    check_host_load(strict=getattr(args, "strict", False))
    out = _run_dir(_resolve_root(args, root), "scaling")
    counts = [int(x) for x in args.entities.split(",")]
    refreshes = [int(x) for x in args.refresh_ms.split(",")]
    cells, status = [], "ok"
    first_meta = {}
    for refresh in refreshes:
        for ec in counts:
            reps = []
            for rep in range(args.repeats):
                try:
                    cell = _syn_cell(ec, args, out, f"r{refresh}_e{ec}_{rep}")
                    reps.append(cell)
                    if not first_meta:
                        first_meta = cell.get("_meta", {})
                except Exception as exc:
                    status = "partial"
                    (out / f"r{refresh}_e{ec}_{rep}.error").write_text(str(exc))
            if reps:
                cell = _aggregate_steady(reps)
                cell["refresh_ms"] = refresh
                cells.append(cell)
            (out / "partial.json").write_text(json.dumps({"status": status, "cells": cells},
                                                          indent=2, default=str))
    rows = scaling_rows(cells)

    # B2: fit one power-law per refresh group, never pool across refresh rates
    from collections import defaultdict
    by_refresh = defaultdict(list)
    for r in rows:
        by_refresh[r.get("refresh_ms", "?")].append(r)

    verdicts, fits = [], []
    for refresh_val, group in sorted(by_refresh.items()):
        g_ecs = [r["entity_count"] for r in group]
        g_uss = [r["uss_kib_median"] for r in group]
        v, f = report.scaling_verdict(g_ecs, g_uss)
        f["refresh_ms"] = refresh_val
        verdicts.append(f"refresh={refresh_val}ms: {v}")
        fits.append(f)

    # Use the first fit as the primary for scaling markdown header; chart uses all groups.
    fit = fits[0] if fits else {"exponent": 0, "r2": 0, "ci_lo": 0, "ci_hi": 0}
    verdict = "; ".join(verdicts)
    # Report the swept refresh value (first entry) as the applied refresh_ms.
    applied_refresh = refreshes[0] if refreshes else "unknown"
    meta = _write_metadata(out, args, first_meta, applied_refresh)
    (out / "summary.json").write_text(json.dumps(
        {"status": status, "rows": rows, "fits": fits, "fit": fit, "verdict": verdict},
        indent=2, default=str))
    (out / "report.md").write_text(report.render_scaling_markdown(rows, verdict, fit, meta=meta))
    # Build per-group (rows, fit) pairs for the chart so each series uses its own fit.
    groups_with_fits = []
    for refresh_val, group in sorted(by_refresh.items()):
        group_fit = next((f for f in fits if f.get("refresh_ms") == refresh_val), fit)
        groups_with_fits.append((group, group_fit))
    report.render_scaling_chart(groups_with_fits, str(out / "scaling.png"))
    if getattr(args, "validate_against_demo", False):
        demo = turtlebot3.TURTLEBOT3
        dfile = str(REPO / demo.compose_file)
        denv = dict(os.environ, COMPOSE_PROFILES=demo.compose_profile)
        dcell = runner.run_cell(dfile, "bench_val", demo.gateway_service, demo.gateway_proc,
                                demo.api_base, GATEWAY_PORT, denv, args.duration,
                                args.interval, str(out / "demo_val.csv"))
        demo_ec = max(1, dcell.get("entity_count", 1))
        demo_upe = dcell.get("uss_kib", 0) / demo_ec
        with (out / "report.md").open("a") as fh:
            fh.write("\n" + report.validate_synthetic(demo_upe, rows, demo_ec) + "\n")
    print(f"scaling: {status} :: {verdict} -> {out}")


PARAMS_BAKE = Path(__file__).resolve().parent / "_params.yaml"  # gitignored


def load_override_sets(path):
    return yaml.safe_load(Path(path).read_text())


def _materialize(demo, override):
    base = yaml.safe_load((REPO / demo.base_params_rel).read_text())
    return yaml.safe_dump(apply_override_at(base, demo.override_root, override))


def cmd_sweep(args, root=None):
    check_host_load(strict=getattr(args, "strict", False))
    out = _run_dir(_resolve_root(args, root), "sweep")
    sets = load_override_sets(
        str(Path(__file__).resolve().parent / "configs" / "overrides.yaml"))
    variants = {"default": {}, **sets}
    cells, status = [], "ok"
    first_meta = {}
    # For sweep the refresh value comes from the baked params; use the default label.
    sweep_refresh_ms = _read_demo_refresh_ms(REPO / turtlebot3.TURTLEBOT3.base_params_rel)
    for label, override in variants.items():
        PARAMS_BAKE.write_text(_materialize(turtlebot3.TURTLEBOT3, override))
        reps = []
        for rep in range(args.repeats):
            try:
                env = dict(os.environ, BENCH_ENTITIES=str(args.entities),
                           BENCH_PARAMS_FILE="/ws/benchmark/_params.yaml")
                cell = runner.run_cell(SYN_COMPOSE, f"bench_sweep_{label}_{rep}", "bench",
                                       "gateway_node", "/api/v1", GATEWAY_PORT, env,
                                       args.duration, args.interval,
                                       str(out / f"{label}_{rep}.csv"))
                reps.append(cell)
                if not first_meta:
                    first_meta = cell.get("_meta", {})
            except Exception as exc:
                status = "partial"
                (out / f"{label}_{rep}.error").write_text(str(exc))
        if reps:
            cell = _aggregate_steady(reps)
            cell["label"] = label
            cells.append(cell)
        (out / "partial.json").write_text(json.dumps({"status": status, "cells": cells},
                                                      indent=2, default=str))
    sweep_meta = _write_metadata(out, args, first_meta, sweep_refresh_ms)
    (out / "summary.json").write_text(json.dumps({"status": status, "cells": cells},
                                                  indent=2, default=str))
    (out / "report.md").write_text(report.render_sweep_markdown(cells, meta=sweep_meta))
    report.render_sweep_bar(cells, str(out / "sweep.png"))
    print(f"sweep: {status} -> {out}")


def cmd_heap(args, root=None):
    check_host_load(strict=getattr(args, "strict", False))
    out = _run_dir(_resolve_root(args, root), "heap")
    env = dict(os.environ, TOOL="heaptrack", BENCH_ENTITIES=str(args.entities))
    project = "bench_heap"
    dh.compose_up(SYN_COMPOSE, project, env=env, service="bench")
    status = "ok"
    heap_meta = {}
    try:
        container = dh.service_container(SYN_COMPOSE, project, "bench")
        if not runner.ready_in(container, GATEWAY_PORT, "/api/v1"):
            raise RuntimeError("gateway never became ready")
        pid = dh.resolve_gateway_pid(container)
        clk = dh.getconf_clk_tck(container)
        read = lambda p, f: dh.read_proc(container, p, f)  # noqa: E731
        # Capture per-cell meta while the container is alive.
        try:
            heap_meta["image_digest"] = dh.image_digest_of_container(container)
        except Exception:
            heap_meta["image_digest"] = "unknown"
        try:
            heap_meta["gateway_sha"] = dh.read_gateway_sha(container)
        except Exception:
            heap_meta["gateway_sha"] = "unknown"
        try:
            heap_meta["allocator"] = dh.read_allocator_from_maps(container, pid)
        except Exception:
            heap_meta["allocator"] = "unknown"
        # A4: run the warmup gate so the slope is measured on settled memory, not startup ramp
        warmup_converged = runner._warmup_gate(read, pid, container, GATEWAY_PORT, "/api/v1")
        samples = sampler.sample_window(read, pid, clk, lambda: 0.0,
                                        args.duration, args.interval, str(out / "heap.csv"))
        dh.run(["docker", "exec", container, "kill", "-INT", "1"])
        time.sleep(8)
        # pipefail makes the pipeline fail if heaptrack_print fails (without it
        # the exit code is tail's, which is 0 even on empty input). bash (not the
        # default dash sh) is required for `set -o pipefail`.
        raw = dh.run(["docker", "exec", container, "bash", "-c",
                      "set -o pipefail; heaptrack_print /tmp/heaptrack.gateway.* "
                      "| tail -n 200"])
        # Fail loud if the trace was not finalized: an empty/summary-less output
        # would otherwise parse as zero leaked / zero sites and report clean.
        if ("total memory leaked" not in raw
                and "peak heap memory consumption" not in raw):
            raise RuntimeError(
                "heaptrack_print produced no summary (trace not finalized?); "
                "rerun with a longer settle or inspect the container logs")
        summary = parse_heaptrack_summary(raw)
        ci = slope_ci95([s.t for s in samples], [s.uss_kib * 1024 for s in samples])
        # B4: honest verdict - only SUSPECTED when warmup converged AND sites attributed
        has_sites = bool(summary.top_sites)
        verdict = report.leak_verdict(ci, summary.total_leaked_bytes,
                                      warmup_converged=warmup_converged,
                                      has_sites=has_sites)
        heap_run_meta = _write_metadata(out, args, heap_meta, refresh_ms="unknown")
        (out / "report.md").write_text(
            report.render_heap_markdown(summary, ci, verdict, meta=heap_run_meta))
        (out / "summary.json").write_text(json.dumps(
            {"status": status, "verdict": verdict, "warmup_converged": warmup_converged,
             "total_leaked": summary.total_leaked_bytes}, indent=2))
        print(f"heap: {status} :: {verdict} -> {out}")
    except Exception as exc:
        status = "failed"
        (out / "error.txt").write_text(str(exc))
        print(f"heap: failed -> {out}: {exc}")
    finally:
        dh.compose_down(SYN_COMPOSE, project)


def cmd_memcheck(args, root=None):
    check_host_load(strict=getattr(args, "strict", False))
    out = _run_dir(_resolve_root(args, root), "memcheck")
    env = dict(os.environ, TOOL="valgrind", BENCH_ENTITIES=str(args.entities))
    project = "bench_memcheck"
    dh.compose_up(SYN_COMPOSE, project, env=env, service="bench")
    memcheck_meta = {}
    try:
        container = dh.service_container(SYN_COMPOSE, project, "bench")
        # Gate on readiness: under valgrind the gateway is slow to start, but a
        # crashed/early-exiting gateway must be reported as an error, not parsed
        # as a clean run. Allow a generous timeout for the valgrind slowdown.
        if not runner.ready_in(container, GATEWAY_PORT, "/api/v1", timeout_s=900):
            raise RuntimeError("gateway never became ready under valgrind")
        try:
            memcheck_meta["image_digest"] = dh.image_digest_of_container(container)
        except Exception:
            memcheck_meta["image_digest"] = "unknown"
        try:
            memcheck_meta["gateway_sha"] = dh.read_gateway_sha(container)
        except Exception:
            memcheck_meta["gateway_sha"] = "unknown"
        memcheck_meta["allocator"] = "glibc"  # valgrind replaces allocator
        time.sleep(args.settle)
        # valgrind is PID 1 (it wraps the gateway executable directly), so SIGINT
        # triggers a clean shutdown and the leak walk.
        dh.run(["docker", "exec", container, "kill", "-INT", "1"])
        # valgrind walks the whole heap on exit (slow for a DDS process), so poll
        # for the LEAK SUMMARY rather than guessing a fixed sleep. The summary
        # goes to stderr, which `docker logs` forwards on the stderr stream, so
        # capture it with merge_stderr. Fail loud if it never finalizes instead
        # of parsing empty output as a clean (0, 0) run.
        logs = ""
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            logs = dh.run(["docker", "logs", container], merge_stderr=True)
            if "LEAK SUMMARY" in logs:
                break
            time.sleep(5)
        if "LEAK SUMMARY" not in logs:
            raise RuntimeError("valgrind did not emit a LEAK SUMMARY (the gateway "
                               "did not finalize under valgrind); the lane cannot "
                               "report leak data")
        s = parse_memcheck_summary(logs)
        _write_metadata(out, args, memcheck_meta, refresh_ms="unknown")
        (out / "report.md").write_text(report.render_memcheck_markdown(s))
        (out / "summary.json").write_text(json.dumps(
            {"status": "ok", "definitely_lost": s.definitely_lost_bytes}, indent=2))
        print(f"memcheck: definitely lost {s.definitely_lost_bytes} B -> {out}")
    finally:
        dh.compose_down(SYN_COMPOSE, project)


def cmd_attribute(args, root=None):
    out = _run_dir(_resolve_root(args, root), "attribute")
    env = dict(os.environ, TOOL="heaptrack", BENCH_ENTITIES=str(args.attribute_at))
    project = "bench_attr"
    dh.compose_up(SYN_COMPOSE, project, env=env, service="bench")
    try:
        container = dh.service_container(SYN_COMPOSE, project, "bench")
        if not runner.ready_in(container, GATEWAY_PORT, "/api/v1"):
            raise RuntimeError("gateway never became ready")
        time.sleep(args.settle)
        dh.run(["docker", "exec", container, "kill", "-INT", "1"])
        time.sleep(8)
        raw = dh.run(["docker", "exec", container, "sh", "-c",
                      "heaptrack_print --print-allocations /tmp/heaptrack.gateway.* | tail -n 200"])
        summary = parse_heaptrack_summary(raw)
        lines = [f"# Swell attribution at N={args.attribute_at}", "",
                 "Top allocation call-sites:", ""]
        for b, site in summary.top_sites:
            lines.append(f"- {b / 1024 / 1024:.2f} MiB :: {site}")
        (out / "report.md").write_text("\n".join(lines) + "\n")
        print(f"attribute: -> {out}")
    finally:
        dh.compose_down(SYN_COMPOSE, project)


def cmd_load(args, root=None):
    """Measure gateway footprint under off/light/heavy HTTP load levels.

    The load generator runs INSIDE the synthetic gateway container
    (DooD-correct: the orchestrator cannot reach the container over the
    network).  For each load level the function:

    1. Brings up the synthetic gateway+graph container.
    2. Starts the load generator in the background inside the container.
    3. Samples the gateway footprint for args.duration seconds.
    4. Reads the load_stats.json back from the container.
    5. Reports USS/CPU/threads + p50/p95 latency vs level.

    Reachability note: ``/ws/benchmark`` must be present in the benchmark
    image (it is - the Dockerfile copies the harness there).  The demo
    turtlebot3 image does NOT have /ws/benchmark; running ``footprint
    --load`` against the demo container falls back to no latency stats
    (load_stats will be empty).  Use ``load`` against the synthetic
    substrate for latency data.
    """
    check_host_load(strict=getattr(args, "strict", False))
    out = _run_dir(_resolve_root(args, root), "load")
    levels = ["off", "light", "heavy"]
    cells, status = [], "ok"
    first_meta = {}

    for level in levels:
        reps = []
        for rep in range(args.repeats):
            tag = f"l{level}_r{rep}"
            env = dict(os.environ, BENCH_ENTITIES=str(args.entities),
                       BENCH_TOPICS_PER_NODE=str(args.topics_per_node),
                       BENCH_MSG_TYPES=str(args.msg_types))
            try:
                cell = runner.run_cell(
                    SYN_COMPOSE, f"bench_load_{tag}", "bench", "gateway_node",
                    "/api/v1", GATEWAY_PORT, env, args.duration, args.interval,
                    str(out / f"{tag}.csv"),
                    load_level=level, load_duration=args.duration)
                if not first_meta:
                    first_meta = cell.get("_meta", {})
                cell["level"] = level
                reps.append(cell)
                (out / f"{tag}.json").write_text(json.dumps(cell, indent=2, default=str))
            except Exception as exc:
                status = "partial"
                (out / f"{tag}.error").write_text(str(exc))

        if reps:
            agg = _aggregate_steady(reps)
            agg["label"] = level
            agg["level"] = level
            # Aggregate latency stats: median p50 and median p95 across all
            # reps that produced load_stats.  Using the last rep only would
            # give a single-sample number without any cross-rep robustness.
            import statistics as _stats
            ls_list = [r["load_stats"] for r in reps if r.get("load_stats")]
            p50_vals = [ls["p50_ms"] for ls in ls_list if "p50_ms" in ls]
            p95_vals = [ls["p95_ms"] for ls in ls_list if "p95_ms" in ls]
            agg["p50_ms"] = _stats.median(p50_vals) if p50_vals else 0.0
            agg["p95_ms"] = _stats.median(p95_vals) if p95_vals else 0.0
            # Thread census from THIS level's own reps (not the first lane's
            # meta): thread counts change with load, so reusing the off-level
            # census would misreport the breakdown under heavy load.
            level_meta = next((r.get("_meta", {}) for r in reps if r.get("_meta")), {})
            agg["threads"] = level_meta.get("threads", {})
            cells.append(agg)

        (out / "partial.json").write_text(json.dumps({"status": status, "cells": cells},
                                                      indent=2, default=str))

    _write_metadata(out, args, first_meta, refresh_ms="unknown")
    (out / "summary.json").write_text(json.dumps({"status": status, "cells": cells},
                                                  indent=2, default=str))
    (out / "report.md").write_text(report.render_load_markdown(cells))
    print(f"load: {status} -> {out}")


def _fault_burst(container, n, mode, out, tag, args):
    """Run one (N, mode) burst measurement cell.

    Brings up the fault-enabled container (already up; caller manages lifecycle),
    waits for warmup, injects N faults, returns the summarize_burst dict and
    rosbag_got count.
    """
    from statistics import median as _med
    from benchmark.lib.burst import sample_burst, summarize_burst

    pid = dh.resolve_gateway_pid(container, _FAULT_PROC)
    clk = dh.getconf_clk_tck(container)
    read = lambda p, f: dh.read_proc(container, p, f)  # noqa: E731
    host_load = lambda: os.getloadavg()[0]  # noqa: E731

    # Warmup: wait for gateway to be ready and USS to settle before sampling.
    if not runner.ready_in(container, GATEWAY_PORT, "/api/v1"):
        raise RuntimeError("gateway never became ready")
    warmup_ok = runner._warmup_gate(read, pid, container, GATEWAY_PORT, "/api/v1")
    if not warmup_ok:
        # Gateway did not reach steady state; mark so the caller can flag the
        # cell as not-steady rather than silently measuring a ramping baseline.
        raise RuntimeError("warmup gate did not converge before fault burst")

    csv_path = str(out / f"{tag}.csv")
    trigger_fn = lambda: inject_faults(container, n, dh.run)  # noqa: E731

    samples, t_trigger = sample_burst(
        read_fn=read,
        pid=pid,
        clk_tck=clk,
        host_load_fn=host_load,
        pre_s=args.pre_s,
        window_s=args.window_s,
        interval=args.interval,
        csv_path=csv_path,
        trigger_fn=trigger_fn,
    )

    # Baseline: median USS over pre-trigger samples.
    pre_samples = [s for s in samples if s.t < t_trigger]
    if not pre_samples:
        pre_samples = samples[:1]
    baseline_uss_kib = float(_med([s.uss_kib for s in pre_samples]))

    burst_result = summarize_burst(
        samples=samples,
        t_trigger=t_trigger,
        baseline_uss_kib=baseline_uss_kib,
        window_s=args.window_s,
        clk_tck=clk,
    )

    # Count rosbag WARNs from container logs.
    rosbag_got = 0
    if mode == "rosbag":
        try:
            import subprocess
            log_result = subprocess.run(
                ["docker", "logs", container],
                capture_output=True, text=True,
            )
            log_text = log_result.stdout + log_result.stderr
            from benchmark.lib.fault_injector import count_rosbag_warns
            rosbag_got = count_rosbag_warns(log_text, n)
        except Exception:
            rosbag_got = 0

    return burst_result, rosbag_got


def cmd_fault(args, root=None):
    """Fault/snapshot burst lane: measure snapshot capture cost vs N concurrent faults.

    For each N in --faults, runs two modes (data-snapshot only, data+rosbag).
    The synthetic substrate brings up gateway_node + fault_manager_node in one
    container.  The demo substrate (--demo turtlebot3) injects via Scripts API
    when a suitable script is available.

    Metrics per (N, mode): peak USS delta, peak CPU, capture duration,
    residual, recovered, rosbag_got/N.
    """
    check_host_load(strict=getattr(args, "strict", False))
    out = _run_dir(_resolve_root(args, root), "fault")
    n_list = [int(x) for x in args.faults.split(",")]
    modes = ["data", "rosbag"]

    burst_results = {}
    rosbag_counts = {}
    status = "ok"
    first_meta = {}

    for mode in modes:
        rosbag_env = "1" if mode == "rosbag" else "0"
        env = dict(os.environ, BENCH_ROSBAG=rosbag_env)
        project = f"bench_fault_{mode}"

        for n in n_list:
            # Each (N, mode) cell gets a FRESH container so fault_manager
            # in-memory state (SQLite + rosbag buffer + heap) does not
            # accumulate across N values.  Without this, the pre-burst
            # baseline for N=4 already holds N=1+N=2 faults and
            # peak_uss_delta is not comparable across N.
            dh.compose_down(SYN_COMPOSE, project)
            dh.compose_up(SYN_COMPOSE, project, env=env, service=_FAULT_SERVICE)
            try:
                container = dh.service_container(SYN_COMPOSE, project, _FAULT_SERVICE)
                # Wait for the gateway HTTP to be ready (so gateway_node exists
                # and the fault_manager has come up) before resolving the PID.
                if not runner.ready_in(container, GATEWAY_PORT, "/api/v1"):
                    raise RuntimeError("gateway never became ready")
                # Capture metadata once from the first live container.
                if not first_meta:
                    try:
                        pid0 = dh.resolve_gateway_pid(container, _FAULT_PROC)
                        first_meta["image_digest"] = dh.image_digest_of_container(container)
                        first_meta["gateway_sha"] = dh.read_gateway_sha(container)
                        first_meta["allocator"] = dh.read_allocator_from_maps(container, pid0)
                    except Exception:
                        pass

                tag = f"m{mode}_n{n}"
                try:
                    burst, rosbag_got = _fault_burst(container, n, mode, out, tag, args)
                    burst_results[(n, mode)] = burst
                    rosbag_counts[(n, mode)] = rosbag_got
                    (out / f"{tag}.json").write_text(
                        json.dumps({"n": n, "mode": mode,
                                    "burst": burst, "rosbag_got": rosbag_got},
                                   indent=2, default=str))
                except Exception as exc:
                    status = "partial"
                    (out / f"{tag}.error").write_text(str(exc))
            finally:
                dh.compose_down(SYN_COMPOSE, project)

    rows = build_fault_rows(n_list, modes, burst_results, rosbag_counts)
    meta = _write_metadata(out, args, first_meta, refresh_ms="n/a")
    (out / "summary.json").write_text(
        json.dumps({"status": status, "rows": rows}, indent=2, default=str))
    if rows:
        (out / "report.md").write_text(report.render_fault_markdown(rows, meta))
        report.render_fault_chart(rows, str(out / "fault.png"))
    print(f"fault: {status} -> {out}")


def cmd_churn(args, root=None):
    """Leak-under-graph-churn gate (no heaptrack, self-contained PASS/FAIL).

    Runs the synthetic gateway twice: against a STATIC graph (control) and against
    a CHURNING graph (bounded node count, topology recycled). Compares the
    steady-window USS slope. A gateway that frees what it rebuilds keeps the churn
    slope at the static level; a churn slope well above static means memory grows
    per graph change. Exits 1 when a churn-driven leak is detected, so the fix can
    be validated with one command (slope must fall to the static level).
    """
    check_host_load(strict=getattr(args, "strict", False))
    out = _run_dir(_resolve_root(args, root), "churn")
    base_env = dict(os.environ, BENCH_ENTITIES=str(args.entities),
                    BENCH_TOPICS_PER_NODE=str(args.topics_per_node),
                    BENCH_MSG_TYPES=str(args.msg_types))

    static = runner.run_cell(
        SYN_COMPOSE, "bench_churn_static", "bench", "gateway_node", "/api/v1",
        GATEWAY_PORT, dict(base_env, BENCH_CHURN_SEC="0"),
        args.duration, args.interval, str(out / "static.csv"))
    static["label"] = "static"

    churn = runner.run_cell(
        SYN_COMPOSE, "bench_churn_on", "bench", "gateway_node", "/api/v1",
        GATEWAY_PORT, dict(base_env, BENCH_CHURN_SEC=str(args.churn_sec),
                           BENCH_CHURN_COUNT=str(args.churn_count)),
        args.duration, args.interval, str(out / "churn.csv"))
    churn["label"] = "churn"

    is_leak, verdict = report.churn_verdict(static, churn)
    meta = _write_metadata(out, args, churn.get("_meta", {}), refresh_ms="n/a")
    (out / "summary.json").write_text(json.dumps(
        {"status": "ok", "leak": is_leak, "verdict": verdict,
         "static": _strip(static), "churn": _strip(churn)}, indent=2, default=str))
    (out / "report.md").write_text(
        report.render_churn_markdown(static, churn, verdict, meta=meta))
    print(f"churn: {'LEAK' if is_leak else 'PASS'} :: {verdict} -> {out}")
    if is_leak:
        raise SystemExit(1)


def cmd_all(args, root=None):
    shared = root or _resolve_root(args, None) or RESULTS / time.strftime("%Y%m%d-%H%M%S")
    import copy as _copy
    cmd_footprint(args, root=shared)
    cmd_scaling(args, root=shared)
    sweep_args = _copy.copy(args)
    sweep_args.entities = int(args.entities.split(",")[-1])
    cmd_sweep(sweep_args, root=shared)
    heap_args = _copy.copy(args)
    heap_args.entities = int(args.entities.split(",")[-1])
    heap_args.duration, heap_args.interval = 600, 5
    cmd_heap(heap_args, root=shared)
    fault_args = _copy.copy(args)
    fault_args.faults = _DEFAULT_FAULTS
    fault_args.pre_s = 10.0
    fault_args.window_s = 60.0
    fault_args.interval = 0.2
    cmd_fault(fault_args, root=shared)
    cmd_report(argparse.Namespace(run=shared.name), root=shared)


def _latest_run_dir():
    """Return the most recent results dir, or raise a clear error if there is none.

    Guards against a missing or empty benchmark/results/ (which would otherwise
    raise FileNotFoundError / IndexError).
    """
    if not RESULTS.is_dir():
        raise RuntimeError(f"no results directory yet: {RESULTS} (run a lane first)")
    candidates = sorted(p for p in RESULTS.iterdir() if p.is_dir())
    if not candidates:
        raise RuntimeError(f"no results found in {RESULTS} (run a lane first)")
    return candidates[-1]


def _find_run_dir(run_arg):
    """Resolve --run arg: 'latest' -> most recent results dir, else join with RESULTS."""
    if run_arg == "latest":
        return _latest_run_dir()
    p = Path(run_arg)
    if p.is_absolute():
        return p
    return RESULTS / run_arg


def cmd_compare(args):
    from benchmark.lib.compare import load_baseline, host_matches, extract_metrics, diff

    run_dir = _find_run_dir(args.run)
    baseline_path = Path(args.baseline)

    if not baseline_path.exists():
        print(f"ERROR: baseline not found: {baseline_path}", flush=True)
        raise SystemExit(2)

    baseline = load_baseline(baseline_path)

    # Collect EVERY lane's run_metadata.json. high_host_load is written per lane,
    # so a noisy run on any single lane must gate the whole comparison - reading
    # only the first lane would let a noisy lane through ungated.
    lane_metas = []
    for lane_dir in sorted(run_dir.iterdir()):
        meta_path = lane_dir / "run_metadata.json"
        if meta_path.exists():
            lane_metas.append((lane_dir.name, json.loads(meta_path.read_text())))

    high_lanes = [name for name, m in lane_metas if m.get("high_host_load")]
    if high_lanes:
        print("ERROR: run was taken under high host load (high_host_load=true on "
              f"lane(s): {', '.join(high_lanes)}) - comparison would be "
              "unreliable.", flush=True)
        raise SystemExit(2)

    # Host fingerprint is per machine, identical across lanes - use any lane.
    run_meta = lane_metas[0][1] if lane_metas else {}
    ok, reason = host_matches(baseline, run_meta)
    if not ok:
        print(f"ERROR: host mismatch - {reason}\n"
              "Cannot compare runs from different hardware.", flush=True)
        raise SystemExit(2)

    any_regression = False
    all_rows = []

    for lane_dir in sorted(run_dir.iterdir()):
        if not lane_dir.is_dir():
            continue
        summary_path = lane_dir / "summary.json"
        if not summary_path.exists():
            continue
        lane = lane_dir.name
        summary = json.loads(summary_path.read_text())
        metrics = extract_metrics(lane, summary)
        if not metrics:
            continue
        rows = diff(baseline, lane, metrics)
        all_rows.extend(rows)

    if not all_rows:
        print("No comparable lanes found in run.")
        return

    header = f"{'Metric':<40} {'Baseline':>12} {'New':>12} {'Delta%':>8} {'Verdict'}"
    print(header)
    print("-" * len(header))
    for row in all_rows:
        bv = f"{row['baseline_val']:.4g}" if row['baseline_val'] is not None else "N/A"
        nv = f"{row['new_val']:.4g}" if row['new_val'] is not None else "N/A"
        dp = f"{row['delta_pct']:+.1f}%" if row['delta_pct'] is not None else "N/A"
        verdict = row["verdict"]
        print(f"{row['metric']:<40} {bv:>12} {nv:>12} {dp:>8} {verdict}")
        if verdict == "REGRESSION":
            any_regression = True

    if any_regression:
        print("\nFAILED: one or more metrics regressed beyond threshold.")
        raise SystemExit(1)
    else:
        print("\nPASSED: no regressions detected.")


def cmd_update_baseline(args):
    from benchmark.lib.compare import extract_metrics

    run_dir = _find_run_dir(args.run)

    run_meta = {}
    for lane_dir in sorted(run_dir.iterdir()):
        meta_path = lane_dir / "run_metadata.json"
        if meta_path.exists():
            run_meta = json.loads(meta_path.read_text())
            break

    # gateway_sha can differ per lane (synthetic image vs demo image), so collect
    # every lane's value and describe the real source instead of a generic guess.
    lane_shas = {}
    for lane_dir in sorted(run_dir.iterdir()):
        meta_path = lane_dir / "run_metadata.json"
        if lane_dir.is_dir() and meta_path.exists():
            lane_shas[lane_dir.name] = json.loads(
                meta_path.read_text()).get("gateway_sha", "unknown")
    real_shas = sorted({s for s in lane_shas.values() if s != "unknown"})
    gateway_sha = real_shas[0] if real_shas else "unknown"
    if lane_shas and all(s != "unknown" for s in lane_shas.values()) and len(real_shas) == 1:
        comment = (
            f"Baseline from run {run_dir.name}. gateway_sha {gateway_sha} is the "
            "measured gateway commit for every lane in this run."
        )
    elif lane_shas:
        per_lane = ", ".join(f"{k}={v}" for k, v in sorted(lane_shas.items()))
        comment = (
            f"Baseline from run {run_dir.name}. gateway_sha per lane: {per_lane}. "
            "Top-level gateway_sha is the first measured commit; 'unknown' means "
            "that lane's image predated SHA capture."
        )
    else:
        comment = (
            f"Baseline from run {run_dir.name}. gateway_sha unknown "
            "(no lane run_metadata.json found)."
        )

    baseline: dict = {
        "_comment": comment,
        "gateway_sha": gateway_sha,
        "host": {
            "cpu_model": run_meta.get("cpu_model", "unknown"),
            "nproc": run_meta.get("nproc", 0),
            "mem_total_kb": run_meta.get("mem_total_kb", 0),
        },
    }

    for lane_dir in sorted(run_dir.iterdir()):
        if not lane_dir.is_dir():
            continue
        summary_path = lane_dir / "summary.json"
        if not summary_path.exists():
            continue
        lane = lane_dir.name
        summary = json.loads(summary_path.read_text())
        metrics = extract_metrics(lane, summary)
        if metrics:
            baseline[lane] = {k: v for k, v in metrics.items() if v is not None}

    out_dir = Path(__file__).resolve().parent / "baseline"
    out_dir.mkdir(exist_ok=True)

    out_path = Path(args.output) if args.output else out_dir / f"{args.name}.json"
    out_path.write_text(json.dumps(baseline, indent=2))
    print(f"Baseline written: {out_path}")


def cmd_report(args, root=None):
    run_dir = root or (RESULTS / args.run if args.run else _latest_run_dir())
    parts = [p / "report.md" for p in sorted(run_dir.iterdir()) if (p / "report.md").exists()]
    (run_dir / "REPORT.md").write_text("\n\n---\n\n".join(p.read_text() for p in parts))
    print(f"combined report -> {run_dir / 'REPORT.md'}")


def main():
    p = argparse.ArgumentParser(prog="benchmark")
    p.add_argument("--strict", action="store_true",
                   help="Abort the lane if host load exceeds cpu_count at lane start.")
    p.add_argument("--run-dir", dest="run_dir", default=None,
                   help="Write this lane into benchmark/results/<run-dir> instead "
                        "of a fresh timestamp dir, so multiple lanes can share one "
                        "run dir for a single 'compare'.")
    sub = p.add_subparsers(dest="cmd", required=True)

    def common(sp):
        sp.add_argument("--demo", default="turtlebot3")
        sp.add_argument("--duration", type=float, default=300)
        sp.add_argument("--interval", type=float, default=1)
        sp.add_argument("--repeats", type=int, default=5)

    fp = sub.add_parser("footprint")
    common(fp)
    fp.add_argument("--load", choices=["off", "light", "heavy"], default="off",
                    help="Run an in-container HTTP load generator during the sample window.")
    fp.set_defaults(func=cmd_footprint)

    sc = sub.add_parser("scaling")
    common(sc)
    sc.add_argument("--entities", default="10,50,100,200")
    sc.add_argument("--refresh-ms", dest="refresh_ms", default="1000")
    sc.add_argument("--topics-per-node", dest="topics_per_node", type=int, default=2)
    sc.add_argument("--msg-types", dest="msg_types", type=int, default=3)
    sc.add_argument("--validate-against-demo", dest="validate_against_demo",
                    action="store_true")
    sc.set_defaults(func=cmd_scaling)

    sw = sub.add_parser("sweep")
    common(sw)
    sw.add_argument("--entities", type=int, default=50)
    sw.set_defaults(func=cmd_sweep)

    hp = sub.add_parser("heap")
    common(hp)
    hp.add_argument("--entities", type=int, default=50)
    hp.set_defaults(func=cmd_heap, duration=1800, interval=5)

    mc = sub.add_parser("memcheck")
    mc.add_argument("--entities", type=int, default=10)
    mc.add_argument("--settle", type=float, default=30)
    mc.set_defaults(func=cmd_memcheck)

    at = sub.add_parser("attribute")
    at.add_argument("--attribute-at", dest="attribute_at", type=int, default=200)
    at.add_argument("--settle", type=float, default=30)
    at.set_defaults(func=cmd_attribute)

    ld = sub.add_parser("load")
    common(ld)
    ld.add_argument("--entities", type=int, default=50)
    ld.add_argument("--topics-per-node", dest="topics_per_node", type=int, default=2)
    ld.add_argument("--msg-types", dest="msg_types", type=int, default=3)
    ld.set_defaults(func=cmd_load)

    fl = sub.add_parser("fault",
                        help="Fault/snapshot burst lane: measure capture cost vs N faults.")
    fl.add_argument("--faults", default=_DEFAULT_FAULTS,
                    help="Comma-separated list of N values to test (default: %(default)s).")
    fl.add_argument("--pre-s", dest="pre_s", type=float, default=10.0,
                    help="Baseline sampling duration before trigger (seconds).")
    fl.add_argument("--window-s", dest="window_s", type=float, default=60.0,
                    help="Post-trigger sampling window (seconds).")
    fl.add_argument("--interval", type=float, default=0.2,
                    help="Sample interval (seconds); default 0.2 for burst resolution.")
    fl.add_argument("--demo", default="synthetic",
                    help="Substrate: 'synthetic' (default) or 'turtlebot3'.")
    fl.set_defaults(func=cmd_fault)

    ch = sub.add_parser(
        "churn",
        help="Leak-under-graph-churn gate: static vs churning graph USS slope; "
             "exit 1 if memory grows under churn.")
    common(ch)
    ch.add_argument("--entities", type=int, default=50)
    ch.add_argument("--topics-per-node", dest="topics_per_node", type=int, default=2)
    ch.add_argument("--msg-types", dest="msg_types", type=int, default=3)
    ch.add_argument("--churn-sec", dest="churn_sec", type=float, default=2.0,
                    help="Recycle the churn pool every N seconds.")
    ch.add_argument("--churn-count", dest="churn_count", type=int, default=10,
                    help="Nodes recycled per churn cycle.")
    ch.set_defaults(func=cmd_churn, duration=600, interval=10)

    al = sub.add_parser("all")
    common(al)
    al.add_argument("--entities", default="10,50,100,200")
    al.add_argument("--refresh-ms", dest="refresh_ms", default="1000")
    al.add_argument("--topics-per-node", dest="topics_per_node", type=int, default=2)
    al.add_argument("--msg-types", dest="msg_types", type=int, default=3)
    al.add_argument("--validate-against-demo", dest="validate_against_demo",
                    action="store_true")
    al.set_defaults(func=cmd_all)

    rp = sub.add_parser("report")
    rp.add_argument("--run", default=None)
    rp.set_defaults(func=cmd_report)

    cmp = sub.add_parser("compare",
                         help="Compare a run against a stored baseline; exit 1 on regression.")
    cmp.add_argument("--run", required=True,
                     help="Run dir timestamp or 'latest'.")
    cmp.add_argument("--baseline", required=True,
                     help="Path to baseline JSON (e.g. benchmark/baseline/ci.json).")
    cmp.set_defaults(func=cmd_compare)

    ub = sub.add_parser("update-baseline",
                        help="Write a new baseline JSON from an existing run (explicit only).")
    ub.add_argument("--run", required=True,
                    help="Run dir timestamp or 'latest'.")
    ub.add_argument("--name", default="ci",
                    help="Baseline name (written to benchmark/baseline/<name>.json).")
    ub.add_argument("--output", default=None,
                    help="Override output path (default: benchmark/baseline/<name>.json).")
    ub.set_defaults(func=cmd_update_baseline)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
