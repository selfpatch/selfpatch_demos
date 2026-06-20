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
import subprocess
import sys
from pathlib import Path

# Repo root = two levels up from this file (benchmark/tests/ -> benchmark/ -> repo).
# Derived from __file__ so the test runs on any checkout / in CI, not just one
# developer's machine.
_REPO_ROOT = Path(__file__).resolve().parents[2]


def test_help_lists_subcommands():
    out = subprocess.run(
        [sys.executable, "-m", "benchmark.benchmark", "-h"],
        capture_output=True, text=True,
        cwd=str(_REPO_ROOT),
    )
    for c in ("footprint", "scaling", "sweep", "heap", "memcheck", "attribute",
              "load", "fault", "churn", "all", "report", "compare", "update-baseline"):
        assert c in out.stdout, out.stdout
