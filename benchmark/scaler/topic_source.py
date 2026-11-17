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
"""A node that publishes one fixed-size String topic: the data a fault captures.

Node name ``bench_source`` in the root namespace, so a fault reported with
source ``/bench_source`` captures ``/bench_source/state`` by entity default.
"""
from __future__ import annotations
import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--payload-bytes", type=int, default=1024)
    ap.add_argument("--hz", type=float, default=10.0)
    args, ros_args = ap.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = Node("bench_source")
    pub = node.create_publisher(String, "/bench_source/state", 10)
    msg = String()
    msg.data = "x" * args.payload_bytes
    node.create_timer(1.0 / args.hz, lambda: pub.publish(msg))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
