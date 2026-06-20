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
"""Pin the valgrind suppression file to the canonical one-token-per-line format.

valgrind FATALs at parse time ("expected '{' or end-of-file") if a '{' or '}'
shares a line with any other token, and the run aborts before the gateway even
starts. With the memcheck lane's readiness gate that surfaces as a hard error;
without it (the old behavior) it silently parsed empty logs as a clean run. This
test keeps the file parseable so the memcheck lane stays functional.
"""
from pathlib import Path

SUPP = Path(__file__).resolve().parents[1] / "profiles" / "fastdds.supp"


def _code_lines():
    return [ln for ln in SUPP.read_text().splitlines()
            if ln.strip() and not ln.lstrip().startswith("#")]


def test_suppression_file_canonical_valgrind_format():
    depth = blocks = memcheck = 0
    for ln in _code_lines():
        s = ln.strip()
        if "{" in s:
            assert s == "{", f"'{{' must be alone on its line: {ln!r}"
            depth += 1
            blocks += 1
        elif "}" in s:
            assert s == "}", f"'}}' must be alone on its line: {ln!r}"
            depth -= 1
        elif s.startswith("Memcheck:"):
            memcheck += 1
        assert depth in (0, 1), f"unexpected brace nesting at {ln!r}"
    assert depth == 0, "unbalanced braces in suppression file"
    assert blocks >= 1, "no suppression blocks found"
    assert memcheck == blocks, "each block needs exactly one Memcheck:<type> line"
