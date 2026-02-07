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

"""
Manipulation Monitor Node for MoveIt 2 Panda Demo.

Monitors robot manipulation for faults and reports to FaultManager:
- Planning failure (MoveGroup action aborted) -> MOTION_PLANNING_FAILED (ERROR)
- Trajectory execution failure -> TRAJECTORY_EXECUTION_FAILED (ERROR)
- Controller timeout -> CONTROLLER_TIMEOUT (ERROR)
- Joint limit approaching/violation -> JOINT_LIMIT_APPROACHING (WARN)
                                    -> JOINT_LIMIT_VIOLATED (ERROR)

Reports faults via /fault_manager/report_fault service.
"""

from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatusArray, GoalStatus
from sensor_msgs.msg import JointState
from ros2_medkit_msgs.srv import ReportFault


# Severity levels (from ReportFault.srv)
SEVERITY_INFO = 0
SEVERITY_WARN = 1
SEVERITY_ERROR = 2
SEVERITY_CRITICAL = 3

# Event types
EVENT_FAILED = 0
EVENT_PASSED = 1

# Panda arm joint limits (from URDF)
PANDA_JOINT_LIMITS = {
    'panda_joint1': (-2.8973, 2.8973),
    'panda_joint2': (-1.7628, 1.7628),
    'panda_joint3': (-2.8973, 2.8973),
    'panda_joint4': (-3.0718, -0.0698),
    'panda_joint5': (-2.8973, 2.8973),
    'panda_joint6': (-0.0175, 3.7525),
    'panda_joint7': (-2.8973, 2.8973),
}

# How close to the limit before warning (radians)
JOINT_LIMIT_WARN_MARGIN = 0.1  # ~5.7 degrees


class ManipulationMonitor(Node):
    """Monitors MoveIt 2 manipulation for faults."""

    def __init__(self):
        super().__init__('manipulation_monitor')

        # Parameters
        self.declare_parameter('joint_limit_warn_margin', JOINT_LIMIT_WARN_MARGIN)
        self.declare_parameter('check_interval_sec', 1.0)
        self.declare_parameter('report_throttle_sec', 5.0)

        self.joint_limit_warn_margin = self.get_parameter(
            'joint_limit_warn_margin'
        ).value
        check_interval = self.get_parameter('check_interval_sec').value
        self.report_throttle_sec = self.get_parameter('report_throttle_sec').value

        # --- Subscribers ---

        # QoS for action status (transient local)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        # MoveGroup action status — for planning failures
        self.move_group_status_sub = self.create_subscription(
            GoalStatusArray,
            '/move_group/_action/status',
            self.move_group_status_callback,
            status_qos,
        )

        # Controller action status — for trajectory execution failures
        self.controller_status_sub = self.create_subscription(
            GoalStatusArray,
            '/panda_arm_controller/follow_joint_trajectory/_action/status',
            self.controller_status_callback,
            status_qos,
        )

        # Joint states — for joint limit monitoring
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )

        # --- Fault reporting service client ---
        self.fault_client = self.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        self.get_logger().info(
            'Waiting for /fault_manager/report_fault service...'
        )
        while not self.fault_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Service not available, waiting...')
        self.get_logger().info('Connected to fault_manager service')

        # --- State tracking ---
        self.move_group_goal_status: Dict[str, int] = {}
        self.controller_goal_status: Dict[str, int] = {}
        self.active_faults: set = set()
        # Track which specific joints are in violation/approaching limits
        self.joints_violating: set = set()
        self.joints_approaching: set = set()

        # Throttling
        self.last_report_times: Dict[str, Time] = {}

        # Timer for periodic checks
        self.check_timer = self.create_timer(check_interval, self.periodic_check)

        self.get_logger().info('ManipulationMonitor started')

    # -----------------------------------------------------------------
    # Subscriber callbacks
    # -----------------------------------------------------------------

    def move_group_status_callback(self, msg: GoalStatusArray):
        """Monitor MoveGroup action for planning/execution failures."""
        for status in msg.status_list:
            goal_id = self._goal_id_str(status)
            last = self.move_group_goal_status.get(goal_id)

            if last != status.status:
                self.move_group_goal_status[goal_id] = status.status

                if status.status == GoalStatus.STATUS_ABORTED:
                    self.report_fault(
                        'MOTION_PLANNING_FAILED',
                        SEVERITY_ERROR,
                        f'MoveGroup goal ABORTED — planning or execution '
                        f'failed (goal: {goal_id[:8]})',
                        EVENT_FAILED,
                    )
                elif status.status == GoalStatus.STATUS_SUCCEEDED:
                    self.report_fault(
                        'MOTION_PLANNING_FAILED',
                        SEVERITY_INFO,
                        'MoveGroup goal succeeded',
                        EVENT_PASSED,
                    )

    def controller_status_callback(self, msg: GoalStatusArray):
        """Monitor arm controller for trajectory execution failures."""
        for status in msg.status_list:
            goal_id = self._goal_id_str(status)
            last = self.controller_goal_status.get(goal_id)

            if last != status.status:
                self.controller_goal_status[goal_id] = status.status

                if status.status == GoalStatus.STATUS_ABORTED:
                    self.report_fault(
                        'TRAJECTORY_EXECUTION_FAILED',
                        SEVERITY_ERROR,
                        f'Arm controller ABORTED trajectory '
                        f'(goal: {goal_id[:8]})',
                        EVENT_FAILED,
                    )
                    self.report_fault(
                        'CONTROLLER_TIMEOUT',
                        SEVERITY_ERROR,
                        'Joint trajectory controller timed out or failed',
                        EVENT_FAILED,
                    )
                elif status.status == GoalStatus.STATUS_SUCCEEDED:
                    self.report_fault(
                        'TRAJECTORY_EXECUTION_FAILED',
                        SEVERITY_INFO,
                        'Trajectory execution succeeded',
                        EVENT_PASSED,
                    )
                    self.report_fault(
                        'CONTROLLER_TIMEOUT',
                        SEVERITY_INFO,
                        'Controller operating normally',
                        EVENT_PASSED,
                    )

    def joint_state_callback(self, msg: JointState):
        """Check joint positions against limits."""
        for i, name in enumerate(msg.name):
            if name not in PANDA_JOINT_LIMITS:
                continue

            position = msg.position[i]
            lower, upper = PANDA_JOINT_LIMITS[name]

            dist_to_lower = position - lower
            dist_to_upper = upper - position

            if dist_to_lower < 0 or dist_to_upper < 0:
                # Joint beyond limit
                self.joints_violating.add(name)
                self.joints_approaching.discard(name)
                self.report_fault_throttled(
                    'JOINT_LIMIT_VIOLATED',
                    SEVERITY_ERROR,
                    f'Joint {name} at {position:.3f} rad — '
                    f'BEYOND limits [{lower:.3f}, {upper:.3f}]',
                    EVENT_FAILED,
                )
            elif (
                dist_to_lower < self.joint_limit_warn_margin
                or dist_to_upper < self.joint_limit_warn_margin
            ):
                # Joint approaching limit
                self.joints_approaching.add(name)
                self.joints_violating.discard(name)
                margin = min(dist_to_lower, dist_to_upper)
                self.report_fault_throttled(
                    'JOINT_LIMIT_APPROACHING',
                    SEVERITY_WARN,
                    f'Joint {name} at {position:.3f} rad — '
                    f'approaching limit (margin: {margin:.3f} rad)',
                    EVENT_FAILED,
                )
            else:
                # Joint is within safe range — remove from tracking sets
                self.joints_violating.discard(name)
                self.joints_approaching.discard(name)

        # Only clear faults when ALL joints are within safe range
        if (
            not self.joints_violating
            and 'JOINT_LIMIT_VIOLATED' in self.active_faults
        ):
            self.report_fault(
                'JOINT_LIMIT_VIOLATED',
                SEVERITY_INFO,
                'All joint positions within limits',
                EVENT_PASSED,
            )
        if (
            not self.joints_approaching
            and 'JOINT_LIMIT_APPROACHING' in self.active_faults
        ):
            self.report_fault(
                'JOINT_LIMIT_APPROACHING',
                SEVERITY_INFO,
                'All joint positions within safe range',
                EVENT_PASSED,
            )

    # -----------------------------------------------------------------
    # Periodic check
    # -----------------------------------------------------------------

    def periodic_check(self):
        """Periodic health checks (placeholder for future checks)."""
        pass  # Could check for stale joint states, etc.

    # -----------------------------------------------------------------
    # Fault reporting helpers
    # -----------------------------------------------------------------

    def report_fault_throttled(
        self, fault_code, severity, description, event_type
    ):
        """Report fault with throttling to avoid flooding."""
        now = self.get_clock().now()
        last = self.last_report_times.get(fault_code)
        if last is not None:
            elapsed = (now - last).nanoseconds / 1e9
            if elapsed < self.report_throttle_sec:
                return
        self.last_report_times[fault_code] = now
        self.report_fault(fault_code, severity, description, event_type)

    def report_fault(self, fault_code, severity, description, event_type):
        """Report a fault to FaultManager via service call."""
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = severity
        request.description = description
        request.source_id = self.get_fully_qualified_name()

        if event_type == EVENT_FAILED:
            self.active_faults.add(fault_code)
        else:
            self.active_faults.discard(fault_code)

        future = self.fault_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_response(f, fault_code, event_type)
        )

    def _handle_response(self, future, fault_code, event_type):
        """Handle response from fault_manager service."""
        try:
            response = future.result()
            if not response.accepted:
                self.get_logger().warn(
                    f'Fault report rejected: {fault_code}'
                )
        except Exception as e:
            self.get_logger().error(
                f'Failed to report fault {fault_code}: {e}'
            )

    @staticmethod
    def _goal_id_str(status) -> str:
        """Convert goal UUID to hex string."""
        return ''.join(f'{b:02x}' for b in status.goal_info.goal_id.uuid)


def main(args=None):
    rclpy.init(args=args)
    node = ManipulationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            'ManipulationMonitor interrupted, shutting down.'
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
