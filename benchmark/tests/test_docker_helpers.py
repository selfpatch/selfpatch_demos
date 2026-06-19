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
import pytest
from benchmark.lib.docker_helpers import parse_pgrep


def test_pgrep_single():
    assert parse_pgrep("4242\n") == 4242


def test_pgrep_zero():
    with pytest.raises(RuntimeError, match="no gateway_node"):
        parse_pgrep("\n")


def test_pgrep_multiple():
    with pytest.raises(RuntimeError, match="multiple"):
        parse_pgrep("4242\n4243\n")
