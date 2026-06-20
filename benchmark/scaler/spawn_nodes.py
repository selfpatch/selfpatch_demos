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
"""Synthetic ROS 2 graph generator parameterized on the gateway cost drivers."""
from __future__ import annotations

_MSG_TYPES = ["std_msgs/msg/String", "std_msgs/msg/Int32", "geometry_msgs/msg/Twist",
              "sensor_msgs/msg/Imu", "sensor_msgs/msg/LaserScan"]


def node_spec(name, topics_per_node, services_per_node,
              msg_type_cardinality, params_per_node, topic_prefix):
    """Build one node spec. ``topic_prefix`` namespaces its topics/services so
    distinct nodes (incl. churn-recycled ones) never collide."""
    types = _MSG_TYPES[:max(1, min(msg_type_cardinality, len(_MSG_TYPES)))]
    return {
        "name": name,
        "topics": [{"name": f"{topic_prefix}/t{j}", "type": types[j % len(types)]}
                   for j in range(topics_per_node)],
        "services": [f"{topic_prefix}/s{j}" for j in range(services_per_node)],
        "params": {f"p{j}": j for j in range(params_per_node)},
    }


def plan_graph(entities, topics_per_node, services_per_node,
               msg_type_cardinality, params_per_node):
    return [
        node_spec(f"bench_node_{i}", topics_per_node, services_per_node,
                  msg_type_cardinality, params_per_node, topic_prefix=f"/bench/n{i}")
        for i in range(entities)
    ]
