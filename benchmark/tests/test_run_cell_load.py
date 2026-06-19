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
"""Tests for run_cell load-generator lifecycle wiring.

Verifies that:
- run_cell with load_level set starts the in-container generator BEFORE
  sample_window (i.e. while the container is still up) and stores load_stats
  in the returned cell.
- run_cell without load_level (existing callers: scaling, sweep) is unaffected
  and still returns an empty load_stats dict.
- run_cell with load_level="off" also leaves load_stats empty (no generator).
"""
from __future__ import annotations
import json
from unittest.mock import MagicMock, call, patch

from benchmark.lib.metrics import Sample

_FAKE_CONTAINER = "abc123"
_FAKE_PID = 42
_FAKE_CLK = 100


def _make_sample(i):
    return Sample(float(i), 1000 + i, 800, 1200, i * 100, 13, 0.0)


def _fake_samples():
    return [_make_sample(i) for i in range(10)]


def _patch_dh(dh_mock, load_module_present=True):
    """Configure the docker_helpers mock for a successful run_cell call."""
    dh_mock.compose_down.return_value = None
    dh_mock.compose_up.return_value = None
    dh_mock.service_container.return_value = _FAKE_CONTAINER
    dh_mock.resolve_gateway_pid.return_value = _FAKE_PID
    dh_mock.getconf_clk_tck.return_value = _FAKE_CLK
    dh_mock.read_proc.return_value = ""
    dh_mock.curl_status_body.return_value = (200, '{"items": []}')
    dh_mock.image_digest_of_container.return_value = "sha256:fake"
    dh_mock.read_gateway_sha.return_value = "deadbeef"
    dh_mock.read_allocator_from_maps.return_value = "glibc"
    dh_mock.thread_census.return_value = {"total": 5, "gateway": 1, "other": 4}

    check_reply = "ok" if load_module_present else "missing"

    def fake_run(argv, env=None):
        cmd = " ".join(str(a) for a in argv)
        # Probe for load_gen.py presence
        if "test -f /ws/benchmark/scaler/load_gen.py" in cmd:
            return check_reply
        # Detached load generator launch - returns empty stdout
        if "docker" in cmd and "exec" in cmd and "-d" in cmd:
            return ""
        # cat /tmp/load_stats.json
        if "cat /tmp/load_stats.json" in cmd:
            return json.dumps({"p50_ms": 12.3, "p95_ms": 45.1, "level": "light"})
        # Default: empty
        return ""

    dh_mock.run.side_effect = fake_run


def _run_cell_with_mocked_dh(dh_mock, **kwargs):
    """Call runner.run_cell with docker_helpers and sampler mocked out."""
    import benchmark.lib.runner as runner_mod
    fake_samples = _fake_samples()

    with patch.object(runner_mod, "dh", dh_mock), \
         patch("benchmark.lib.sampler.sample_window", return_value=fake_samples), \
         patch("benchmark.lib.sampler.per_sample_cores", return_value=[0.1] * 3):
        # Also patch ready_in and _warmup_gate to return immediately.
        with patch.object(runner_mod, "ready_in", return_value=True), \
             patch.object(runner_mod, "_warmup_gate", return_value=True):
            return runner_mod.run_cell(
                "compose.yml", "proj", "bench", "gateway_node",
                "/api/v1", 8080, {}, 30.0, 1.0, "/tmp/test.csv",
                **kwargs,
            )


def test_run_cell_no_load_leaves_load_stats_empty():
    """Existing no-load callers (scaling, sweep) get empty load_stats."""
    dh_mock = MagicMock()
    _patch_dh(dh_mock)

    cell = _run_cell_with_mocked_dh(dh_mock)

    assert cell["load_stats"] == {}
    # Verify the load generator was NOT started (no detached exec call).
    for c in dh_mock.run.call_args_list:
        argv = c.args[0] if c.args else c.kwargs.get("argv", [])
        assert "-d" not in argv, "detached exec should not be called without load_level"


def test_run_cell_load_level_off_leaves_load_stats_empty():
    """load_level='off' must not start the generator."""
    dh_mock = MagicMock()
    _patch_dh(dh_mock)

    cell = _run_cell_with_mocked_dh(dh_mock, load_level="off")

    assert cell["load_stats"] == {}
    for c in dh_mock.run.call_args_list:
        argv = c.args[0] if c.args else c.kwargs.get("argv", [])
        assert "-d" not in argv


def test_run_cell_load_level_light_starts_generator():
    """load_level='light' must start the in-container generator and return stats."""
    dh_mock = MagicMock()
    _patch_dh(dh_mock, load_module_present=True)

    cell = _run_cell_with_mocked_dh(dh_mock, load_level="light", load_duration=30.0)

    # load_stats populated from the fake cat output
    assert cell["load_stats"].get("p50_ms") == 12.3
    assert cell["load_stats"].get("p95_ms") == 45.1

    # Verify a detached exec was issued (the load generator launch).
    detached_calls = [
        c for c in dh_mock.run.call_args_list
        if "-d" in (c.args[0] if c.args else [])
    ]
    assert len(detached_calls) == 1, "exactly one detached exec expected"
    launch_cmd = detached_calls[0].args[0]
    assert "load_gen.py" in " ".join(launch_cmd)
    assert "--level light" in " ".join(launch_cmd)


def test_run_cell_load_module_absent_returns_empty_stats():
    """When /ws/benchmark is absent, load_stats is empty and no detached exec fires."""
    dh_mock = MagicMock()
    _patch_dh(dh_mock, load_module_present=False)

    cell = _run_cell_with_mocked_dh(dh_mock, load_level="light", load_duration=30.0)

    assert cell["load_stats"] == {}
    detached_calls = [
        c for c in dh_mock.run.call_args_list
        if "-d" in (c.args[0] if c.args else [])
    ]
    assert len(detached_calls) == 0


def test_run_cell_load_generator_called_before_compose_down():
    """The generator must be started while the container is still up.

    Verifies the call order: detached exec -> cat stats all happen before
    the final compose_down in the finally block.  Uses a unified call_log
    so all events share the same sequence counter.
    """
    dh_mock = MagicMock()
    _patch_dh(dh_mock, load_module_present=True)

    call_log: list = []
    original_run = dh_mock.run.side_effect

    def tracking_run(argv, env=None):
        call_log.append(("run", list(argv)))
        return original_run(argv, env=env)

    def tracking_compose_down(*args, **kwargs):
        call_log.append(("compose_down", list(args)))

    dh_mock.run.side_effect = tracking_run
    dh_mock.compose_down.side_effect = tracking_compose_down

    _run_cell_with_mocked_dh(dh_mock, load_level="light", load_duration=30.0)

    # Find positions of key events in the unified log
    detach_pos = next(
        (i for i, (kind, cmd) in enumerate(call_log)
         if kind == "run" and "-d" in cmd), -1
    )
    cat_pos = next(
        (i for i, (kind, cmd) in enumerate(call_log)
         if kind == "run" and "cat" in " ".join(cmd)
         and "load_stats.json" in " ".join(cmd)), -1
    )
    # run_cell calls compose_down twice: once at the top to clear prior state
    # and once in the finally block for teardown.  We want the LAST one
    # (the teardown) to verify the generator runs before teardown.
    down_positions = [
        i for i, (kind, _) in enumerate(call_log) if kind == "compose_down"
    ]
    assert len(down_positions) >= 1, "compose_down not found in call log"
    final_down_pos = down_positions[-1]

    assert detach_pos != -1, "detached exec not found in call log"
    assert cat_pos != -1, "cat load_stats.json not found in call log"
    assert detach_pos < final_down_pos, "generator must be launched before final compose_down"
    assert cat_pos < final_down_pos, "load_stats must be read before final compose_down"
