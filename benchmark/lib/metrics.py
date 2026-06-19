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
"""Pure numeric helpers. No I/O, no docker."""
from __future__ import annotations

import math
import statistics
from dataclasses import dataclass


@dataclass
class Sample:
    t: float
    uss_kib: int
    pss_kib: int
    rss_kib: int
    total_ticks: int
    num_threads: int
    host_load1: float


def compute_cpu_cores(prev_ticks, curr_ticks, clk_tck, elapsed_s):
    if elapsed_s <= 0 or clk_tck <= 0:
        return 0.0
    return (curr_ticks - prev_ticks) / clk_tck / elapsed_s


def median(values):
    return float(statistics.median(values))


def iqr(values):
    if len(values) < 2:
        v = float(values[0]) if values else 0.0
        return v, v
    q = statistics.quantiles(values, n=4, method="inclusive")
    return float(q[0]), float(q[2])


def linfit(xs, ys):
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    slope = sxy / sxx if sxx else 0.0
    intercept = my - slope * mx
    ss_tot = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot else 1.0
    return slope, intercept, r2


# Student-t critical values at 0.975 (two-tailed 95% CI), indexed by df = 1..30.
# For df > 30 we use the normal approximation 1.96.
_T_TABLE = {
    1: 12.706, 2: 4.303, 3: 3.182, 4: 2.776, 5: 2.571,
    6: 2.447, 7: 2.365, 8: 2.306, 9: 2.262, 10: 2.228,
    11: 2.201, 12: 2.179, 13: 2.160, 14: 2.145, 15: 2.131,
    16: 2.120, 17: 2.110, 18: 2.101, 19: 2.093, 20: 2.086,
    21: 2.080, 22: 2.074, 23: 2.069, 24: 2.064, 25: 2.060,
    26: 2.056, 27: 2.052, 28: 2.048, 29: 2.045, 30: 2.042,
}


def _t_crit(df):
    """Student-t critical value at 0.975 for given degrees of freedom."""
    if df <= 0:
        return 12.706
    df_int = int(df)
    if df_int in _T_TABLE:
        return _T_TABLE[df_int]
    # Interpolate between table entries where possible
    if df_int < 30:
        lo = _T_TABLE.get(df_int, 2.042)
        hi = _T_TABLE.get(df_int + 1, 2.042)
        frac = df - df_int
        return lo + frac * (hi - lo)
    # df > 30: normal approximation is accurate enough
    return 1.96


def _lag1_autocorr(resid):
    """Lag-1 autocorrelation of a sequence (Pearson on consecutive pairs)."""
    n = len(resid)
    if n < 3:
        return 0.0
    mean_r = sum(resid) / n
    c0 = sum((r - mean_r) ** 2 for r in resid)
    if c0 == 0.0:
        return 0.0
    c1 = sum((resid[i] - mean_r) * (resid[i + 1] - mean_r) for i in range(n - 1))
    # Use n (not n-1) in denominator to match the standard biased estimator
    return c1 / c0


def slope_ci95(xs, ys, correct_autocorr=True):
    """Return (slope, ci_lo, ci_hi) for an OLS slope, 95% Student-t CI.

    With correct_autocorr=True (default) the CI is widened for serial dependence:
    the effective sample size is n_eff = n*(1-rho)/(1+rho) where rho is the lag-1
    autocorrelation of the residuals (clamped so n_eff >= 3), the SE is scaled by
    sqrt((n-2)/(n_eff-2)), and t uses df = n_eff - 2. This is for TIME-SERIES
    slopes (consecutive /proc samples are autocorrelated).

    With correct_autocorr=False the points are treated as independent (e.g. a
    scaling fit over distinct graph sizes) and a plain OLS t-CI (df = n-2) is used.
    Applying the autocorrelation correction to independent points would be wrong.
    """
    n = len(xs)
    slope, intercept, _ = linfit(xs, ys)
    mx = sum(xs) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if n <= 2 or sxx == 0:
        return slope, slope, slope
    resid = [y - (slope * x + intercept) for x, y in zip(xs, ys)]
    # OLS residual variance with n-2 dof
    s2 = sum(r ** 2 for r in resid) / (n - 2)
    se_ols = math.sqrt(s2 / sxx)

    if correct_autocorr:
        rho = _lag1_autocorr(resid)
        rho = max(-0.999, min(0.999, rho))
        n_eff = max(3.0, n * (1.0 - rho) / (1.0 + rho))
        df_eff = max(1.0, n_eff - 2.0)
        se = se_ols * math.sqrt((n - 2) / df_eff)
    else:
        df_eff = n - 2
        se = se_ols

    t = _t_crit(df_eff)
    return slope, slope - t * se, slope + t * se


def loglog_exponent(xs, ys):
    """Return (exp, r2, ci_lo, ci_hi) from a log-log OLS fit with 95% CI.

    Uses slope_ci95 with correct_autocorr=False (independent points - distinct
    graph sizes are not a time series) and a plain OLS t-CI (df = n-2).

    When fewer than 3 valid points are available the CI collapses to the point
    estimate, which would produce a false CONFIRMED verdict.  In that case
    return (-inf, +inf) for the CI bounds so the caller emits INDETERMINATE.
    """
    pts = [(math.log(x), math.log(y)) for x, y in zip(xs, ys) if x > 0 and y > 0]
    if len(pts) < 2:
        return 0.0, 0.0, 0.0, 0.0
    lx = [p[0] for p in pts]
    ly = [p[1] for p in pts]
    _, _, r2 = linfit(lx, ly)
    # scaling points (distinct graph sizes) are independent, not a time series
    exp, ci_lo, ci_hi = slope_ci95(lx, ly, correct_autocorr=False)
    # With n < 3 the CI collapses to the point estimate (slope_ci95 returns
    # slope, slope, slope when n <= 2).  An interval [exp, exp] would pass
    # either the ci_lo > 1 or ci_hi < 1 guard in scaling_verdict and emit a
    # false CONFIRMED.  Return open bounds so the verdict is INDETERMINATE.
    if len(pts) < 3:
        return exp, r2, float('-inf'), float('inf')
    return exp, r2, ci_lo, ci_hi


def steady_window(samples, frac=1 / 3):
    if not samples:
        return []
    k = max(1, int(len(samples) * frac))
    return samples[-k:]


def peak_uss(samples):
    return max((s.uss_kib for s in samples), default=0)
