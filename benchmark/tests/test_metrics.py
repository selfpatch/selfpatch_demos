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
import math
from benchmark.lib.metrics import (
    Sample, compute_cpu_cores, median, iqr, linfit, slope_ci95,
    loglog_exponent, steady_window, peak_uss,
)


def test_compute_cpu_cores_one_core():
    assert compute_cpu_cores(1000, 1100, 100, 1.0) == 1.0


def test_compute_cpu_cores_two_cores():
    assert compute_cpu_cores(0, 200, 100, 1.0) == 2.0


def test_compute_cpu_cores_zero_elapsed():
    assert compute_cpu_cores(0, 50, 100, 0.0) == 0.0


def test_median_odd_even():
    assert median([3, 1, 2]) == 2
    assert median([1, 2, 3, 4]) == 2.5


def test_iqr_inclusive_quartiles():
    # statistics.quantiles(n=4, method="inclusive") on 1..8 -> (2.75, 6.25)
    q1, q3 = iqr([1, 2, 3, 4, 5, 6, 7, 8])
    assert math.isclose(q1, 2.75) and math.isclose(q3, 6.25)


def test_linfit_perfect_line():
    slope, intercept, r2 = linfit([0, 1, 2, 3], [1, 3, 5, 7])
    assert math.isclose(slope, 2.0) and math.isclose(intercept, 1.0)
    assert math.isclose(r2, 1.0)


def test_slope_ci95_flat_series_includes_zero():
    _, lo, hi = slope_ci95([0, 1, 2, 3, 4], [10, 11, 9, 10, 10])
    assert lo < 0 < hi


def test_loglog_exponent_quadratic_data():
    # y = x^2 -> exponent ~ 2.0; function now returns (exp, r2, ci_lo, ci_hi)
    xs = [10, 20, 40, 80]
    ys = [x * x for x in xs]
    exp, r2, ci_lo, ci_hi = loglog_exponent(xs, ys)
    assert math.isclose(exp, 2.0, abs_tol=0.05) and r2 > 0.99
    # CI must be entirely above 1 for this clean quadratic signal
    assert ci_lo > 1.0


def test_loglog_exponent_linear_data():
    xs = [10, 20, 40, 80]
    ys = [3 * x for x in xs]
    exp, _r2, ci_lo, ci_hi = loglog_exponent(xs, ys)
    assert math.isclose(exp, 1.0, abs_tol=0.05)
    # With only 4 perfectly linear points the CI straddles 1
    assert ci_lo <= 1.0 <= ci_hi


def test_slope_ci95_autocorrelated_flat_straddles_zero():
    """B3: an AR(1) autocorrelated but flat series must give a CI that straddles zero.

    With strong positive autocorrelation and 1.96 hard-coded critical value, the old
    implementation produced an interval that wrongly excluded zero.  The corrected
    n_eff + Student-t implementation must include zero.
    """
    import random
    rng = random.Random(42)
    # AR(1) process with rho=0.95 (strongly autocorrelated) around a flat mean of 1000
    n = 30
    ys = [0.0] * n
    ys[0] = 1000.0
    rho = 0.95
    for i in range(1, n):
        ys[i] = 1000.0 + rho * (ys[i - 1] - 1000.0) + rng.gauss(0, 2)
    xs = list(range(n))
    _, lo, hi = slope_ci95(xs, ys)
    assert lo < 0 < hi, (
        f"CI [{lo:.4f}, {hi:.4f}] does not straddle zero for autocorrelated flat series"
    )


def test_slope_ci95_positive_autocorr_widens_ci():
    """B3: with strongly positive AR(1) residuals, n_eff < n -> wider CI than z=1.96 * se_ols.

    Construct data where the OLS residuals are positively autocorrelated by building a
    flat-mean series from an AR(1) process with rho ~ 0.9.  The implementation must
    produce a half-width wider than what z=1.96 * se_ols (the non-corrected formula)
    would give, because n_eff < n reduces the effective degrees of freedom.
    """
    import math
    # AR(1) residuals, rho=0.9, mean=0 (we add this to a flat baseline)
    # xs are evenly spaced so the OLS residuals track the AR(1) noise exactly.
    n = 10
    # Build residuals with positive correlation: each step nudged in same direction
    ar_resid = [0.0] * n
    ar_resid[0] = 1.0
    for i in range(1, n):
        ar_resid[i] = 0.9 * ar_resid[i - 1]  # pure AR(1) rho=0.9, no added noise
    xs = list(range(n))
    ys = [100.0 + r for r in ar_resid]  # flat line plus AR(1) perturbation
    slope, lo, hi = slope_ci95(xs, ys)
    half_width_corrected = (hi - lo) / 2.0

    # Compare against the naive z=1.96 half-width (pre-correction formula)
    from benchmark.lib.metrics import linfit
    _, intercept, _ = linfit(xs, ys)
    mx = sum(xs) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    resid_ols = [y - (slope * x + intercept) for x, y in zip(xs, ys)]
    s_err = (sum(r ** 2 for r in resid_ols) / (n - 2)) ** 0.5
    se_ols = s_err / math.sqrt(sxx)
    z_half = 1.96 * se_ols

    assert half_width_corrected > z_half, (
        f"corrected half-width {half_width_corrected:.6f} should exceed z-based "
        f"{z_half:.6f} when residuals are positively autocorrelated"
    )


def test_steady_window_last_third():
    s = [Sample(i, i, 0, 0, 0, 1, 0.0) for i in range(9)]
    assert [x.uss_kib for x in steady_window(s)] == [6, 7, 8]


def test_peak_uss():
    s = [Sample(0, u, 0, 0, 0, 1, 0.0) for u in (5, 12, 7)]
    assert peak_uss(s) == 12


def test_loglog_exponent_n_lt3_returns_open_ci():
    """With fewer than 3 valid points the CI must be open (-inf, +inf).

    slope_ci95 collapses to (exp, exp, exp) for n<=2, which would pass the
    ci_lo > 1.0 guard and emit a false CONFIRMED for any exponent > 1.
    The fix must return (-inf, +inf) so scaling_verdict lands in INDETERMINATE.
    """
    # Exactly 2 valid points: clean quadratic -> exponent ~2.0
    xs, ys = [10, 100], [100, 10000]
    exp, r2, ci_lo, ci_hi = loglog_exponent(xs, ys)
    assert math.isinf(ci_lo) and ci_lo < 0, "ci_lo must be -inf for n=2"
    assert math.isinf(ci_hi) and ci_hi > 0, "ci_hi must be +inf for n=2"
    # The exponent point estimate itself should still be reasonable
    assert math.isclose(exp, 2.0, abs_tol=0.1)


def test_loglog_exponent_n_eq1_returns_zero():
    """A single point returns all zeros (degenerate case)."""
    exp, r2, ci_lo, ci_hi = loglog_exponent([10], [100])
    assert exp == 0.0 and r2 == 0.0 and ci_lo == 0.0 and ci_hi == 0.0


def test_loglog_exponent_n_eq3_has_finite_ci():
    """With exactly 3 valid points the CI should be finite (df=1, wide but finite)."""
    xs, ys = [10, 20, 40], [100, 400, 1600]
    exp, r2, ci_lo, ci_hi = loglog_exponent(xs, ys)
    assert math.isfinite(ci_lo) and math.isfinite(ci_hi)
    # 3 perfectly quadratic points -> exponent 2.0
    assert math.isclose(exp, 2.0, abs_tol=0.1)
