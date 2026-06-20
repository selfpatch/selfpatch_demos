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
from pathlib import Path
import benchmark.lib.sampler as sampler_mod
from benchmark.lib.sampler import sample_once, per_sample_cores
from benchmark.lib.metrics import Sample

FIX = Path(__file__).parent / "fixtures"

_FNAMES = {"smaps_rollup": "smaps_rollup.txt", "status": "status.txt",
           "stat": "stat.txt"}


def _read(pid, fname):
    return (FIX / _FNAMES[fname]).read_text()


class _FakeClock:
    """Deterministic clock for sample_window: monotonic() reads the clock and
    sleep() advances it, so the sampling loop is fully reproducible (no wall
    time)."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, s):
        self.t += s


def test_sample_once_fields():
    s = sample_once(_read, 4242, 0.7)
    assert s.uss_kib == 20000 and s.pss_kib == 42000 and s.rss_kib == 81920
    assert s.total_ticks == 660 and s.num_threads == 13 and s.host_load1 == 0.7


def test_per_sample_cores_between_consecutive():
    a = Sample(0.0, 1, 1, 1, 0, 1, 0.0)
    b = Sample(1.0, 2, 2, 2, 100, 1, 0.0)
    c = Sample(2.0, 3, 3, 3, 250, 1, 0.0)
    assert per_sample_cores([a, b, c], 100) == [1.0, 1.5]


def test_sample_window_transient_read_error_does_not_lose_window(monkeypatch, tmp_path):
    """A single /proc read hiccup is skipped, not fatal: the rest of the window
    is still collected instead of the whole repeat being lost."""
    monkeypatch.setattr(sampler_mod, "time", _FakeClock())
    calls = {"n": 0}

    def read(pid, fname):
        # Fail only the very first read of the window, then recover.
        if calls["n"] == 0 and fname == "smaps_rollup":
            calls["n"] += 1
            raise OSError("transient /proc read")
        return _read(pid, fname)

    samples = sampler_mod.sample_window(
        read, 4242, 100, lambda: 0.0, duration=1.0, interval=0.2,
        csv_path=str(tmp_path / "w.csv"))
    # First tick errored and was skipped; the remaining ticks produced samples.
    assert len(samples) >= 3


def test_sample_window_non_consecutive_errors_never_break(monkeypatch, tmp_path):
    """Errors below the consecutive threshold, separated by a success, must never
    end the window: the counter resets on each good read (pattern fail,fail,ok,
    fail,fail,ok... never reaches 3 in a row)."""
    monkeypatch.setattr(sampler_mod, "time", _FakeClock())
    fail_ticks = {0, 1, 3, 4}  # successes at ticks 2 and 5 reset the run of 2
    state = {"tick": 0}

    def read(pid, fname):
        if fname == "smaps_rollup":
            i = state["tick"]
            state["tick"] += 1
            if i in fail_ticks:
                raise OSError("transient")
        return _read(pid, fname)

    samples = sampler_mod.sample_window(
        read, 4242, 100, lambda: 0.0, duration=1.4, interval=0.2,
        csv_path=str(tmp_path / "w.csv"))
    # The window ran to completion (never broke at 2 consecutive); the successful
    # ticks were collected.
    assert len(samples) >= 1


def test_sample_window_process_gone_breaks_and_summarizes(monkeypatch, tmp_path):
    """When reads fail persistently (process genuinely gone) the window stops
    after the consecutive-error threshold instead of raising or hanging."""
    monkeypatch.setattr(sampler_mod, "time", _FakeClock())

    def read(pid, fname):
        raise OSError("no such process")

    samples = sampler_mod.sample_window(
        read, 4242, 100, lambda: 0.0, duration=60.0, interval=0.2,
        csv_path=str(tmp_path / "w.csv"))
    # No exception propagated; nothing collected.
    assert samples == []
