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
"""Baseline load, metric extraction and comparison logic for regression tracking."""
from __future__ import annotations
import json
from pathlib import Path
from typing import Any


# Threshold definitions per metric: (warn_pct, regression_pct).
# scaling.exponent uses ci_lo > 1.0 as the regression gate; the pct thresholds
# are stored but the exponent row is handled separately in diff().
_THRESHOLDS: dict[str, tuple[float, float]] = {
    "footprint.uss_kib_median": (5.0, 10.0),
    "footprint.cpu_cores_median": (15.0, 30.0),
    "footprint.num_threads_median": (15.0, 30.0),
    "scaling.exponent": (15.0, float("inf")),  # ci_lo crossing 1.0 is the regression gate
    "load.heavy_uss_kib_median": (5.0, 10.0),
    "load.heavy_p95_ms": (20.0, 50.0),
}


def load_baseline(path: str | Path) -> dict:
    """Load a baseline JSON file. Raises FileNotFoundError if missing."""
    return json.loads(Path(path).read_text())


def host_matches(baseline: dict, run_meta: dict) -> tuple[bool, str]:
    """Return (matches, reason) comparing host fingerprint.

    Matches when cpu_model and nproc are identical.
    """
    bh = baseline.get("host", {})
    if bh.get("cpu_model") != run_meta.get("cpu_model"):
        return False, (
            f"cpu_model mismatch: baseline={bh.get('cpu_model')!r} "
            f"run={run_meta.get('cpu_model')!r}"
        )
    if bh.get("nproc") != run_meta.get("nproc"):
        return False, (
            f"nproc mismatch: baseline={bh.get('nproc')} "
            f"run={run_meta.get('nproc')}"
        )
    return True, ""


def extract_metrics(lane: str, summary: dict) -> dict[str, float | None]:
    """Extract comparable scalar metrics from a lane summary.json dict.

    Returns {metric_key: value_or_None}.  None means the metric is absent in
    this summary (single-sample fault lane, partial run, etc.).
    """
    metrics: dict[str, float | None] = {}

    if lane == "footprint":
        cell = summary.get("cell", {})
        metrics["footprint.uss_kib_median"] = cell.get("uss_kib_median")
        metrics["footprint.cpu_cores_median"] = cell.get("cpu_cores_median")
        metrics["footprint.num_threads_median"] = cell.get("num_threads_median")

    elif lane == "scaling":
        fit = summary.get("fit", {})
        metrics["scaling.exponent"] = fit.get("exponent")
        metrics["scaling.ci_lo"] = fit.get("ci_lo")
        metrics["scaling.ci_hi"] = fit.get("ci_hi")

    elif lane == "load":
        cells = summary.get("cells", [])
        heavy = next((c for c in cells if c.get("level") == "heavy"), None)
        if heavy:
            metrics["load.heavy_uss_kib_median"] = heavy.get("uss_kib_median")
            metrics["load.heavy_p95_ms"] = heavy.get("p95_ms")

    elif lane == "fault":
        # Fault is single-sample / informational only - not a regression gate.
        pass

    return metrics


def diff(
    baseline: dict,
    lane: str,
    new_metrics: dict[str, float | None],
) -> list[dict[str, Any]]:
    """Compute per-metric delta rows.

    Returns list of dicts with keys:
      metric, baseline_val, new_val, delta_pct, verdict
    where verdict is one of: OK, WARN, REGRESSION, N/A
    """
    b_lane = baseline.get(lane, {})
    rows = []

    for key, new_val in new_metrics.items():
        b_val = b_lane.get(key)

        if b_val is None or new_val is None:
            rows.append({
                "metric": key,
                "baseline_val": b_val,
                "new_val": new_val,
                "delta_pct": None,
                "verdict": "N/A",
            })
            continue

        # Scaling exponent: special ci_lo > 1.0 regression gate.
        if key == "scaling.exponent":
            ci_lo = new_metrics.get("scaling.ci_lo")
            b_exp = b_val
            delta_pct = ((new_val - b_exp) / b_exp * 100) if b_exp else None
            verdict = "OK"
            if ci_lo is not None and ci_lo > 1.0:
                verdict = "REGRESSION"
            rows.append({
                "metric": key,
                "baseline_val": b_exp,
                "new_val": new_val,
                "delta_pct": delta_pct,
                "verdict": verdict,
            })
            continue

        # Skip ci_lo/ci_hi as standalone rows (used only in exponent check above).
        if key in ("scaling.ci_lo", "scaling.ci_hi"):
            continue

        if b_val == 0:
            delta_pct = None
            verdict = "OK" if new_val == 0 else "WARN"
        else:
            delta_pct = (new_val - b_val) / abs(b_val) * 100
            if key in _THRESHOLDS:
                warn_pct, reg_pct = _THRESHOLDS[key]
                if delta_pct >= reg_pct:
                    verdict = "REGRESSION"
                elif delta_pct >= warn_pct:
                    verdict = "WARN"
                else:
                    verdict = "OK"
            else:
                verdict = "OK"

        rows.append({
            "metric": key,
            "baseline_val": b_val,
            "new_val": new_val,
            "delta_pct": delta_pct,
            "verdict": verdict,
        })

    return rows
