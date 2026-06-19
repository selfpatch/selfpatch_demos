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


def test_help_lists_subcommands():
    out = subprocess.run(
        [sys.executable, "-m", "benchmark.benchmark", "-h"],
        capture_output=True, text=True,
        cwd="/home/bburda/workspace/wt/sd-benchmark",
    )
    for c in ("footprint", "scaling", "sweep", "heap", "memcheck", "attribute", "all", "report"):
        assert c in out.stdout
