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
from benchmark.lib.warmup import entity_count_stable, uss_derivative_below
from benchmark.lib.metrics import Sample


def test_stable_true():
    assert entity_count_stable([5, 8, 12, 12, 12], 3) is True


def test_stable_false():
    assert entity_count_stable([12, 12, 13], 3) is False


def test_stable_rejects_zero_value():
    """A5: a gateway with zero discovered entities is NOT stable for benchmarking."""
    assert entity_count_stable([0, 0, 0], 3) is False


def test_stable_rejects_none_sentinel():
    """A5: None sentinels (all-probe-fail) must not satisfy the stability check."""
    assert entity_count_stable([None, None, None], 3) is False


def test_stable_rejects_mixed_none():
    """A5: a mix of valid and None values is also not stable."""
    assert entity_count_stable([5, None, 5], 3) is False


def _s(t, u):
    return Sample(t, u, 0, 0, 0, 1, 0.0)


def test_deriv_flat():
    assert uss_derivative_below([_s(0, 1000), _s(10, 1002)], 1.0) is True


def test_deriv_climbing():
    assert uss_derivative_below([_s(0, 1000), _s(10, 2000)], 1.0) is False
