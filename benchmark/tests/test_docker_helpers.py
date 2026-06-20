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

import pytest

import benchmark.lib.docker_helpers as dh
from benchmark.lib.docker_helpers import getconf_clk_tck, parse_pgrep


def test_pgrep_single():
    assert parse_pgrep("4242\n") == 4242


def test_pgrep_zero():
    with pytest.raises(RuntimeError, match="no gateway_node"):
        parse_pgrep("\n")


def test_pgrep_multiple():
    with pytest.raises(RuntimeError, match="multiple"):
        parse_pgrep("4242\n4243\n")


class _FakeCompleted:
    def __init__(self, stdout="", stderr="", returncode=0):
        self.stdout = stdout
        self.stderr = stderr
        self.returncode = returncode


def test_run_returns_stdout(monkeypatch):
    monkeypatch.setattr(dh.subprocess, "run",
                        lambda *a, **k: _FakeCompleted(stdout="OUT", stderr="ERR"))
    assert dh.run(["x"]) == "OUT"


def test_run_merge_stderr_appends_stderr(monkeypatch):
    """merge_stderr=True returns stdout+stderr (valgrind writes to stderr)."""
    monkeypatch.setattr(dh.subprocess, "run",
                        lambda *a, **k: _FakeCompleted(stdout="OUT", stderr="ERR"))
    assert dh.run(["x"], merge_stderr=True) == "OUTERR"


def test_run_timeout_raises_runtimeerror(monkeypatch):
    """A subprocess timeout becomes a RuntimeError, not a hang."""
    def boom(*a, **k):
        raise subprocess.TimeoutExpired(cmd="x", timeout=5)
    monkeypatch.setattr(dh.subprocess, "run", boom)
    with pytest.raises(RuntimeError, match="timed out"):
        dh.run(["x"], timeout=5)


def test_run_nonzero_raises(monkeypatch):
    monkeypatch.setattr(dh.subprocess, "run",
                        lambda *a, **k: _FakeCompleted(stderr="bad", returncode=1))
    with pytest.raises(RuntimeError, match="failed"):
        dh.run(["x"])


def test_getconf_clk_tck_accepts_non_100(monkeypatch):
    """CLK_TCK != 100 is valid (some kernels/arches differ); do not hard-fail."""
    monkeypatch.setattr(dh, "run", lambda *a, **k: "250\n")
    assert getconf_clk_tck("c") == 250


def test_getconf_clk_tck_rejects_nonpositive(monkeypatch):
    monkeypatch.setattr(dh, "run", lambda *a, **k: "0\n")
    with pytest.raises(RuntimeError, match="unusable"):
        getconf_clk_tck("c")
