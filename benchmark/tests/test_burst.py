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
"""Unit tests for burst.summarize_burst (pure function, no I/O)."""
import math
from benchmark.lib.burst import summarize_burst
from benchmark.lib.metrics import Sample


def _make_samples(t_values, uss_values, ticks_per_step=0):
    """Build a Sample list from parallel t/uss sequences."""
    samples = []
    acc_ticks = 0
    for t, uss in zip(t_values, uss_values):
        acc_ticks += ticks_per_step
        samples.append(Sample(t=t, uss_kib=uss, pss_kib=0, rss_kib=0,
                               total_ticks=acc_ticks, num_threads=10, host_load1=0.0))
    return samples


def test_clean_spike_recovered():
    """A spike that returns to baseline -> recovered=True, small residual."""
    # baseline at 1000 KiB, spikes to 1500 at t=5, returns by t=8
    ts = [0.0, 1.0, 2.0, 3.0, 4.0,
          5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
    uss = [1000, 1000, 1000, 1000, 1000,
           1500, 1400, 1100, 1010, 1005, 1000]
    samples = _make_samples(ts, uss)
    t_trigger = 5.0
    result = summarize_burst(samples, t_trigger, baseline_uss_kib=1000.0,
                             recover_frac=0.05, window_s=5.0)

    assert result["recovered"] is True
    assert result["peak_uss_delta_kib"] == 500  # 1500 - 1000
    assert result["residual_uss_delta_kib"] == 0  # 1000 - 1000
    # capture_duration <= total window
    assert 0 < result["capture_duration_s"] <= 5.0


def test_elevated_spike_not_recovered():
    """A spike that never returns to baseline -> recovered=False, residual>0."""
    ts = [0.0, 1.0, 2.0, 3.0, 4.0,
          5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
    uss = [1000, 1000, 1000, 1000, 1000,
           1500, 1450, 1400, 1350, 1300, 1300]
    samples = _make_samples(ts, uss)
    t_trigger = 5.0
    result = summarize_burst(samples, t_trigger, baseline_uss_kib=1000.0,
                             recover_frac=0.05, window_s=5.0)

    assert result["recovered"] is False
    assert result["residual_uss_delta_kib"] > 0  # still elevated at end
    assert result["peak_uss_delta_kib"] == 500
    # capture_duration_s equals the full window
    assert math.isclose(result["capture_duration_s"], 5.0, abs_tol=0.01)


def test_peak_delta_correct():
    """peak_uss_delta_kib is max post-trigger minus baseline, not absolute max."""
    # Pre-trigger samples include a higher USS that shouldn't count as peak
    ts = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    uss = [2000, 1000, 1000, 1000, 1200, 1000]
    samples = _make_samples(ts, uss)
    t_trigger = 2.5  # only t=3,4,5 are post-trigger
    result = summarize_burst(samples, t_trigger, baseline_uss_kib=1000.0,
                             recover_frac=0.05, window_s=3.0)
    # Only post-trigger: uss=[1000,1200,1000], peak delta = 200
    assert result["peak_uss_delta_kib"] == 200


def test_peak_cpu_cores_computed():
    """peak_cpu_cores is > 0 when CPU ticks increase after trigger."""
    # 100 ticks/s = 1 core; CLK_TCK hardcoded to 100 in sampler
    ts = [0.0, 1.0, 2.0, 3.0, 4.0]
    uss = [1000, 1000, 1000, 1000, 1000]
    # ticks_per_step=100 -> 1 core per second
    samples = _make_samples(ts, uss, ticks_per_step=100)
    t_trigger = 1.5
    result = summarize_burst(samples, t_trigger, baseline_uss_kib=1000.0,
                             recover_frac=0.05, window_s=3.0)
    # At least one post-trigger pair has cpu=1.0
    assert result["peak_cpu_cores"] > 0.0


def test_no_post_trigger_samples():
    """With no samples after trigger, returns safe zero values."""
    ts = [0.0, 1.0, 2.0]
    uss = [1000, 1000, 1000]
    samples = _make_samples(ts, uss)
    result = summarize_burst(samples, t_trigger=3.0, baseline_uss_kib=1000.0)
    assert result["recovered"] is True
    assert result["peak_uss_delta_kib"] == 0
    assert result["peak_cpu_cores"] == 0.0
    assert result["capture_duration_s"] == 0.0


def test_baseline_uss_kib_in_result():
    """The input baseline_uss_kib is echoed into the result dict."""
    samples = _make_samples([0.0, 1.0], [800, 900])
    result = summarize_burst(samples, t_trigger=0.5, baseline_uss_kib=800.0)
    assert result["baseline_uss_kib"] == 800.0


def test_clk_tck_scales_peak_cpu():
    """peak_cpu_cores uses the passed clk_tck, not a hardcoded 100."""
    ts = [0.0, 1.0, 2.0, 3.0, 4.0]
    uss = [1000, 1000, 1000, 1000, 1000]
    samples = _make_samples(ts, uss, ticks_per_step=100)  # 100 ticks/s
    t_trigger = 1.5
    # clk_tck=100 -> 100 ticks/s == 1.0 core
    r100 = summarize_burst(samples, t_trigger, baseline_uss_kib=1000.0,
                           window_s=3.0, clk_tck=100)
    # clk_tck=200 -> 100 ticks/s == 0.5 core
    r200 = summarize_burst(samples, t_trigger, baseline_uss_kib=1000.0,
                           window_s=3.0, clk_tck=200)
    assert math.isclose(r100["peak_cpu_cores"], 1.0, abs_tol=1e-9)
    assert math.isclose(r200["peak_cpu_cores"], 0.5, abs_tol=1e-9)


def test_recovery_requires_leaving_band_first():
    """USS must rise above the band before recovery is detected: a first
    post-trigger sample still at baseline must not be read as instant recovery."""
    # t_trigger=1.5; post samples: t=2 still at baseline, then a real spike that
    # returns by t=5.
    ts = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    uss = [1000, 1000, 1000, 1500, 1200, 1010, 1000]
    samples = _make_samples(ts, uss)
    result = summarize_burst(samples, t_trigger=1.5, baseline_uss_kib=1000.0,
                             recover_frac=0.05, window_s=4.5)
    assert result["recovered"] is True
    assert result["peak_uss_delta_kib"] == 500
    # Recovery is measured from the trigger to the post-spike return (t=5), NOT
    # the in-band first sample at t=2 -> capture must be well above ~0.
    assert result["capture_duration_s"] > 1.0


def test_no_excursion_is_instant_recovery():
    """A burst whose USS never measurably leaves the band recovers instantly
    (capture 0), not 'never recovered' for the whole window."""
    ts = [0.0, 1.0, 2.0, 3.0, 4.0]
    uss = [1000, 1000, 1001, 1000, 1000]  # within 5% band of 1000
    samples = _make_samples(ts, uss)
    result = summarize_burst(samples, t_trigger=1.5, baseline_uss_kib=1000.0,
                             recover_frac=0.05, window_s=3.0)
    assert result["recovered"] is True
    assert result["capture_duration_s"] == 0.0
