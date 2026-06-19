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
"""Host the synthetic graph: rclpy nodes, generic pubs by type-string."""
from __future__ import annotations
import importlib
import os

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from benchmark.scaler.spawn_nodes import plan_graph


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
    specs = plan_graph(
        int(os.environ.get("BENCH_ENTITIES", "10")),
        int(os.environ.get("BENCH_TOPICS_PER_NODE", "2")),
        int(os.environ.get("BENCH_SERVICES_PER_NODE", "1")),
        int(os.environ.get("BENCH_MSG_TYPES", "3")),
        int(os.environ.get("BENCH_PARAMS_PER_NODE", "3")))
    ex = MultiThreadedExecutor()
    nodes = [SyntheticNode(s) for s in specs]
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
