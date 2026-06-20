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
"""Host the synthetic graph: rclpy nodes, generic pubs by type-string.

Two modes:
- static (default): spawn N nodes once and publish on a timer.
- churn (BENCH_CHURN_SEC > 0): additionally recycle a bounded pool of nodes
  (destroy K + create K fresh ones every BENCH_CHURN_SEC), so the ROS graph
  topology continuously changes while the total node count stays constant. This
  isolates the gateway's handling of graph CHANGES (discovery add/remove +
  re-serialization) - a sustained gateway USS rise under churn, with a flat USS
  under the static graph, points at per-graph-change accumulation in the gateway
  itself rather than at any single workload (e.g. Nav2).
"""
from __future__ import annotations
import importlib
import itertools
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from benchmark.scaler.spawn_nodes import node_spec, plan_graph


def _msg_class(type_str):
    pkg, _, name = type_str.split("/")
    return getattr(importlib.import_module(f"{pkg}.msg"), name)


class SyntheticNode(Node):
    def __init__(self, spec):
        super().__init__(spec["name"])
        self._pubs = []
        for t in spec["topics"]:
            self._pubs.append((self.create_publisher(_msg_class(t["type"]), t["name"], 10),
                               _msg_class(t["type"])))
        for s in spec["services"]:
            self.create_service(Trigger, s, self._noop)
        for k, v in spec["params"].items():
            self.declare_parameter(k, v)
        self.create_timer(0.5, self._tick)

    def _noop(self, _req, resp):
        resp.success = True
        return resp

    def _tick(self):
        for pub, cls in self._pubs:
            try:
                pub.publish(cls())
            except Exception:
                pass


def main():
    rclpy.init()
    tpn = int(os.environ.get("BENCH_TOPICS_PER_NODE", "2"))
    spn = int(os.environ.get("BENCH_SERVICES_PER_NODE", "1"))
    mtc = int(os.environ.get("BENCH_MSG_TYPES", "3"))
    ppn = int(os.environ.get("BENCH_PARAMS_PER_NODE", "3"))
    churn_sec = float(os.environ.get("BENCH_CHURN_SEC", "0"))
    churn_count = int(os.environ.get("BENCH_CHURN_COUNT", "5"))

    specs = plan_graph(int(os.environ.get("BENCH_ENTITIES", "10")),
                       tpn, spn, mtc, ppn)
    ex = MultiThreadedExecutor()
    nodes = [SyntheticNode(s) for s in specs]
    for n in nodes:
        ex.add_node(n)

    if churn_sec <= 0:
        try:
            ex.spin()
        finally:
            for n in nodes:
                n.destroy_node()
            rclpy.shutdown()
        return

    # Churn mode: SINGLE-THREADED. Alternate spin_once with recycling a bounded
    # pool (remove K + add K fresh, brand-new names each cycle) in the SAME
    # thread, so a node is never destroyed while the executor concurrently builds
    # its wait set (that race raises rclpy InvalidHandle). The graph keeps
    # changing but its size stays constant.
    gen = itertools.count(1_000_000)
    pool = []
    next_churn = time.monotonic() + churn_sec
    try:
        while True:
            ex.spin_once(timeout_sec=0.2)
            if time.monotonic() < next_churn:
                continue
            for _ in range(min(churn_count, len(pool))):
                old = pool.pop(0)
                ex.remove_node(old)
                old.destroy_node()
            for _ in range(churn_count):
                i = next(gen)
                node = SyntheticNode(node_spec(
                    f"churn_node_{i}", tpn, spn, mtc, ppn,
                    topic_prefix=f"/bench/churn{i}"))
                ex.add_node(node)
                pool.append(node)
            next_churn = time.monotonic() + churn_sec
    except KeyboardInterrupt:
        pass
    finally:
        for n in nodes + pool:
            try:
                n.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
