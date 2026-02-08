#!/usr/bin/env python3
# Copyright 2026 selfpatch
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

"""Continuous pick-and-place loop for MoveIt 2 Panda demo.

Performs repeating pick-and-place cycles so faults can be injected mid-operation.
Uses MoveGroupInterface via the MoveGroup action for motion planning and execution.

All targets are joint-space goals for guaranteed reachability in both
fake-hardware and Gazebo simulation modes.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

# Joint-space targets (all within Panda limits, collision-free)
PANDA_TARGETS = {
    # "ready" / home — standard upright pose from SRDF
    "ready": {
        "panda_joint1": 0.0,
        "panda_joint2": -0.785,   # -π/4
        "panda_joint3": 0.0,
        "panda_joint4": -2.356,   # -3π/4
        "panda_joint5": 0.0,
        "panda_joint6": 1.571,    # π/2
        "panda_joint7": 0.785,    # π/4
    },
    # "pick" — arm reaching forward and down
    "pick": {
        "panda_joint1": 0.0,
        "panda_joint2": 0.3,
        "panda_joint3": 0.0,
        "panda_joint4": -1.5,
        "panda_joint5": 0.0,
        "panda_joint6": 1.9,
        "panda_joint7": 0.785,
    },
    # "place" — arm reaching to the left
    "place": {
        "panda_joint1": 1.2,
        "panda_joint2": 0.0,
        "panda_joint3": 0.3,
        "panda_joint4": -1.8,
        "panda_joint5": 0.0,
        "panda_joint6": 1.9,
        "panda_joint7": 0.785,
    },
}


class PickPlaceLoop(Node):
    """Repeatedly executes pick-and-place cycles using MoveGroup action."""

    def __init__(self):
        super().__init__("pick_place_loop")

        # Parameters
        self.declare_parameter("cycle_delay_sec", 5.0)
        self.cycle_delay = self.get_parameter("cycle_delay_sec").value

        # MoveGroup action client
        # In MoveIt 2 Jazzy, MoveGroupMoveAction serves at /move_action
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")

        # State machine: pick → place → home → pick ...
        self.cycle_count = 0
        self.phases = ["pick", "place", "home"]
        self.phase_idx = 0

        # State flags
        self.move_group_ready = False
        self.goal_in_flight = False  # Guard against overlapping goals

        # Wait for MoveGroup with periodic retries (non-blocking startup)
        self.get_logger().info("PickPlaceLoop initialized, waiting for MoveGroup...")
        self.startup_timer = self.create_timer(5.0, self.check_move_group_ready)

    def check_move_group_ready(self):
        """Periodically check if MoveGroup action server is available."""
        if self.move_group_client.server_is_ready():
            self.get_logger().info("MoveGroup available, starting pick-and-place loop")
            self.move_group_ready = True
            self.startup_timer.cancel()

            # Start the cycle timer
            self.timer = self.create_timer(self.cycle_delay, self.execute_cycle)
        else:
            self.get_logger().info(
                "MoveGroup not yet available, retrying in 5 seconds..."
            )

    def execute_cycle(self):
        """One pick-and-place cycle step."""
        if not self.move_group_ready:
            return
        if self.goal_in_flight:
            return  # Skip — previous goal still executing

        phase = self.phases[self.phase_idx]
        self.cycle_count += 1
        self.get_logger().info(f"=== Cycle {self.cycle_count} ({phase}) ===")

        # Map phase to joint target
        target_name = "ready" if phase == "home" else phase
        self.send_joint_goal(target_name, phase)

    def send_joint_goal(self, target_name, label):
        """Send a MoveGroup goal using joint-space target values."""
        joint_values = PANDA_TARGETS.get(target_name)
        if not joint_values:
            self.get_logger().error(f"Unknown target: {target_name}")
            return

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # Build joint constraints
        constraints = Constraints()
        for joint_name, position in joint_values.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f"Sending {label} goal (target: {target_name})...")
        self.goal_in_flight = True
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self.goal_response_callback(f, label))

    def goal_response_callback(self, future, label):
        """Handle MoveGroup goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{label} goal REJECTED")
            self.goal_in_flight = False
            return

        self.get_logger().info(f"{label} goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.result_callback(f, label))

    def result_callback(self, future, label):
        """Handle MoveGroup result and advance state machine."""
        result = future.result().result
        status = future.result().status

        self.goal_in_flight = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{label} SUCCEEDED")
            # Advance state machine
            self.phase_idx = (self.phase_idx + 1) % len(self.phases)
        else:
            error_code = result.error_code.val if result else "unknown"
            self.get_logger().warn(
                f"{label} FAILED with status {status}, error_code={error_code}"
            )
            # On failure, retry same phase next cycle


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow clean shutdown on Ctrl+C without printing a traceback.
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
