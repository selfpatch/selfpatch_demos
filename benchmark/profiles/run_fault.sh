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
# no `set -u`: colcon/ament setup.bash reference unbound vars and empty-array
# expansion is fragile under nounset.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

# Build fault_manager params.  confirmation_threshold=-1 means faults confirm
# immediately (no debounce counter), so the snapshot fires on every injected fault.
# snapshots.enabled=true turns on data snapshots.
# snapshots.rosbag.enabled is controlled by BENCH_ROSBAG (default off).
ROSBAG_ENABLED="${BENCH_ROSBAG:-0}"
if [ "${ROSBAG_ENABLED}" = "1" ]; then
    ROSBAG_FLAG="true"
else
    ROSBAG_FLAG="false"
fi

# Start fault_manager_node in background on the same namespace.
ros2 run ros2_medkit_fault_manager fault_manager_node \
    --ros-args \
    -r __ns:=/diagnostics \
    -r __node:=fault_manager \
    -p storage_type:=memory \
    -p confirmation_threshold:=-1 \
    -p snapshots.enabled:=true \
    -p "snapshots.rosbag.enabled:=${ROSBAG_FLAG}" \
    -p snapshots.rosbag.duration_sec:=5.0 \
    -p snapshots.rosbag.format:=mcap &
FM_PID=$!

# Start gateway_node.  No synthetic graph needed for the fault lane - the
# fault_manager is what we're measuring.
ros2 run ros2_medkit_gateway gateway_node \
    --ros-args \
    -r __ns:=/diagnostics \
    -r __node:=ros2_medkit_gateway \
    -p server.host:=0.0.0.0 \
    -p server.port:=8080 &
GW_PID=$!

trap 'kill -INT "${FM_PID}" "${GW_PID}" 2>/dev/null || true' TERM INT

wait "${GW_PID}"
kill -INT "${FM_PID}" 2>/dev/null || true
wait "${FM_PID}" 2>/dev/null || true
