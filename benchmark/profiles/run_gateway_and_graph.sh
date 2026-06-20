#!/usr/bin/env bash
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
# shellcheck disable=SC1091  # ROS/colcon setup.bash is generated at build time
# shellcheck disable=SC2054  # "definite,indirect" is a single valgrind argument
# no `set -u`: colcon/ament setup.bash reference unbound vars (COLCON_TRACE) and
# empty-array expansion is fragile under nounset. We use `${var:-}` defaults instead.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
python3 -m benchmark.scaler.synthetic_graph &
GRAPH_PID=$!
trap 'kill -INT "$GRAPH_PID" 2>/dev/null || true' TERM INT
PARAMS_ARG=()
if [ -n "${BENCH_PARAMS_FILE:-}" ]; then PARAMS_ARG=(--params-file "${BENCH_PARAMS_FILE}"); fi
GW_ARGS=(--ros-args -r __ns:=/diagnostics -r __node:=ros2_medkit_gateway \
  -p server.host:=0.0.0.0 -p server.port:=8080 "${PARAMS_ARG[@]}")
case "${TOOL:-none}" in
  heaptrack)
    # heaptrack follows forked children, so wrapping `ros2 run` (a python
    # launcher that forks the node) still profiles the gateway.
    exec heaptrack -o /tmp/heaptrack.gateway \
      ros2 run ros2_medkit_gateway gateway_node "${GW_ARGS[@]}" ;;
  valgrind)
    # valgrind does NOT trace forked children by default, so wrapping `ros2 run`
    # would profile only the python launcher while the real gateway_node runs as
    # an untraced child (no LEAK SUMMARY for the gateway). Run valgrind DIRECTLY
    # on the gateway executable; resolve its path via `ros2 pkg prefix` so the
    # install layout is not hardcoded.
    GW_EXE="$(ros2 pkg prefix ros2_medkit_gateway)/lib/ros2_medkit_gateway/gateway_node"
    exec valgrind --leak-check=full --show-leak-kinds=definite,indirect \
      --suppressions=/ws/benchmark/profiles/fastdds.supp \
      "$GW_EXE" "${GW_ARGS[@]}" ;;
  *)
    exec ros2 run ros2_medkit_gateway gateway_node "${GW_ARGS[@]}" ;;
esac
