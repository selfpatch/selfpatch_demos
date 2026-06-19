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
"""Sample /proc/<pid>/{smaps_rollup,status,stat}. Parse helpers are pure."""
from __future__ import annotations


def _kib(text, key):
    for line in text.splitlines():
        if line.startswith(key + ":"):
            return int(line.split()[1])
    raise ValueError(f"missing {key}")


def parse_smaps_rollup(text):
    return (_kib(text, "Private_Clean") + _kib(text, "Private_Dirty"),
            _kib(text, "Pss"), _kib(text, "Rss"))


def parse_status(text):
    return _kib(text, "VmRSS"), _kib(text, "Threads")


def parse_stat_ticks(text):
    rest = text[text.rindex(")") + 1:].split()
    return int(rest[11]) + int(rest[12])  # utime(f14)+stime(f15)


import csv  # noqa: E402
import time  # noqa: E402

from benchmark.lib.metrics import Sample, compute_cpu_cores  # noqa: E402

CSV_HEADER = ["t", "uss_kib", "pss_kib", "rss_kib", "total_ticks",
              "num_threads", "host_load1", "cpu_cores"]


def sample_once(read_fn, pid, host_load1):
    uss, pss, rss = parse_smaps_rollup(read_fn(pid, "smaps_rollup"))
    vmrss, threads = parse_status(read_fn(pid, "status"))
    ticks = parse_stat_ticks(read_fn(pid, "stat"))
    return Sample(time.monotonic(), uss, pss, rss, ticks, threads, host_load1)


def write_sample_row(writer, prev, s, clk_tck):
    cores = 0.0 if prev is None else compute_cpu_cores(
        prev.total_ticks, s.total_ticks, clk_tck, s.t - prev.t)
    writer.writerow([s.t, s.uss_kib, s.pss_kib, s.rss_kib, s.total_ticks,
                     s.num_threads, s.host_load1, round(cores, 6)])


def per_sample_cores(samples, clk_tck):
    return [compute_cpu_cores(p.total_ticks, s.total_ticks, clk_tck, s.t - p.t)
            for p, s in zip(samples, samples[1:])]


def sample_window(read_fn, pid, clk_tck, host_load_fn, duration, interval, csv_path):
    samples = []
    with open(csv_path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(CSV_HEADER)
        prev = None
        end = time.monotonic() + duration
        while time.monotonic() < end:
            s = sample_once(read_fn, pid, host_load_fn())
            write_sample_row(w, prev, s, clk_tck)
            fh.flush()
            samples.append(s)
            prev = s
            time.sleep(interval)
    return samples
