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
"""Warmup-gate predicates: entity-count stability + USS-derivative flatness."""
from __future__ import annotations
from benchmark.lib.metrics import linfit


def entity_count_stable(history, window):
    """Return True iff the last `window` entries are all equal AND > 0.

    A gateway that has discovered zero entities is not settled for benchmarking
    even if the count has been consistently zero.
    """
    if len(history) < window:
        return False
    tail = history[-window:]
    # All values must be identical (stable) and the value must be positive.
    if len(set(tail)) != 1:
        return False
    # tail[0] is the stable value; reject None and <= 0
    val = tail[0]
    return val is not None and val > 0


def uss_derivative_below(samples, threshold_kib_s):
    if len(samples) < 2:
        return False
    slope, _, _ = linfit([s.t for s in samples], [s.uss_kib for s in samples])
    return abs(slope) < threshold_kib_s
