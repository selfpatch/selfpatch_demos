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
Anomaly Detector Node for TurtleBot3 Demo

Monitors navigation metrics and reports faults directly to FaultManager:
- Navigation goal ABORTED -> ERROR fault
- Navigation goal CANCELED -> WARN fault
- High AMCL localization uncertainty -> WARN/ERROR fault
- No robot progress when goal active -> WARN fault

Reports faults directly via /fault_manager/report_fault service.
"""

import math
from typing import Optional

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from ros2_medkit_msgs.srv import ReportFault


# Severity levels (from ReportFault.srv)
SEVERITY_INFO = 0
SEVERITY_WARN = 1
SEVERITY_ERROR = 2
SEVERITY_CRITICAL = 3

# Event types
GOAL_STATUS_SOURCE = '/goal_status'

EVENT_FAILED = 0
EVENT_PASSED = 1

# How many PASSED events one recovery sends. The fault manager heals a fault when
# its debounce counter climbs from confirmation_threshold back to
# healing_threshold, so the burst has to be at least the span between them: 3 for
# the debounce profile (-3 to 0), 1 for the goal-status source (-1 to 0). Sending
# a fixed 3 covers both. It is deliberately not unbounded - a fault manager with
# healing disabled keeps a fault CONFIRMED forever, and every further PASSED would
# publish another EVENT_UPDATED onto the fault event stream.
HEAL_PASSED_REPEATS = 3


class AnomalyDetectorNode(Node):
    """Monitors navigation metrics and reports faults to FaultManager."""

    def __init__(self):
        super().__init__('anomaly_detector')

        # Declare parameters
        self.declare_parameter('covariance_warn_threshold', 0.5)
        self.declare_parameter('covariance_error_threshold', 2.0)
        self.declare_parameter('no_progress_timeout_sec', 15.0)
        self.declare_parameter('min_progress_distance', 0.1)
        self.declare_parameter('check_interval_sec', 1.0)

        # Get parameters
        self.covariance_warn_threshold = self.get_parameter('covariance_warn_threshold').value
        self.covariance_error_threshold = self.get_parameter('covariance_error_threshold').value
        self.no_progress_timeout_sec = self.get_parameter('no_progress_timeout_sec').value
        self.min_progress_distance = self.get_parameter('min_progress_distance').value
        check_interval = self.get_parameter('check_interval_sec').value

        # QoS for action status (transient local for late subscribers)
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscribers
        self.goal_status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.goal_status_callback,
            status_qos
        )

        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Service client for fault reporting
        self.fault_client = self.create_client(ReportFault, '/fault_manager/report_fault')
        self.get_logger().info('Waiting for /fault_manager/report_fault service...')
        while not self.fault_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Service not available, waiting...')
        self.get_logger().info('Connected to fault_manager service')

        # Timer for periodic checks
        self.check_timer = self.create_timer(check_interval, self.check_timer_callback)

        # State tracking
        self.last_goal_status: dict = {}  # goal_id -> last_status
        self.has_active_goal = False
        self.last_position: Optional[tuple] = None
        self.last_progress_time: Optional[Time] = None
        self.current_covariance = 0.0

        # Throttling (using ROS time for simulation compatibility)
        self.last_covariance_report_time: Optional[Time] = None
        self.last_no_progress_report_time: Optional[Time] = None
        self.report_throttle_sec = 5.0

        # Codes with PASSED reports still owed to the fault manager.
        # A debounced fault heals only when its counter climbs from the
        # confirmation threshold back to the healing one, which takes as many
        # PASSED events as the span between them. One PASSED per recovery leaves
        # a confirmed fault stuck, so each recovery owes a burst instead.
        self.pending_heal_reports: dict = {}

        self.get_logger().info(
            f'AnomalyDetector started (cov_warn={self.covariance_warn_threshold}, '
            f'cov_error={self.covariance_error_threshold}, '
            f'no_progress_timeout={self.no_progress_timeout_sec}s)'
        )

    def goal_status_callback(self, msg: GoalStatusArray):
        """Monitor navigation goal status for failures."""
        # Compute has_active_goal from entire message to avoid iteration-order issues
        active_statuses = {GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING}
        self.has_active_goal = any(
            s.status in active_statuses for s in msg.status_list
        )

        for status in msg.status_list:
            goal_id = ''.join(f'{b:02x}' for b in status.goal_info.goal_id.uuid)
            last_status = self.last_goal_status.get(goal_id)

            # Track active goals - update progress time when goal is active
            if status.status in active_statuses:
                self.last_progress_time = self.get_clock().now()

            # Detect status changes to terminal failure states
            if last_status != status.status:
                self.last_goal_status[goal_id] = status.status

                if status.status == GoalStatus.STATUS_ABORTED:
                    self.report_fault(
                        fault_code='NAVIGATION_GOAL_ABORTED',
                        source_suffix=GOAL_STATUS_SOURCE,
                        severity=SEVERITY_ERROR,
                        description=f'Navigation goal ABORTED - path planning or execution failed (goal: {goal_id[:8]})',
                        event_type=EVENT_FAILED
                    )
                    self.get_logger().warn(f'Navigation goal {goal_id[:8]} ABORTED')

                elif status.status == GoalStatus.STATUS_CANCELED:
                    self.report_fault(
                        fault_code='NAVIGATION_GOAL_CANCELED',
                        source_suffix=GOAL_STATUS_SOURCE,
                        severity=SEVERITY_WARN,
                        description=f'Navigation goal CANCELED (goal: {goal_id[:8]})',
                        event_type=EVENT_FAILED
                    )
                    self.get_logger().info(f'Navigation goal {goal_id[:8]} CANCELED')

                elif status.status == GoalStatus.STATUS_SUCCEEDED:
                    # Clear navigation faults
                    self.report_fault(
                        fault_code='NAVIGATION_GOAL_ABORTED',
                        source_suffix=GOAL_STATUS_SOURCE,
                        severity=SEVERITY_INFO,
                        description='Navigation goal succeeded',
                        event_type=EVENT_PASSED
                    )
                    self.report_fault(
                        fault_code='NAVIGATION_GOAL_CANCELED',
                        source_suffix=GOAL_STATUS_SOURCE,
                        severity=SEVERITY_INFO,
                        description='Navigation goal succeeded',
                        event_type=EVENT_PASSED
                    )

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Monitor AMCL localization covariance."""
        # Calculate covariance magnitude from position covariance (indices 0, 7 for x, y variance)
        # cov[0] and cov[7] are already variances; use their sum to compute a position stddev magnitude
        cov = msg.pose.covariance
        self.current_covariance = math.sqrt(cov[0] + cov[7])

        now = self.get_clock().now()
        if self.last_covariance_report_time is not None:
            elapsed = (now - self.last_covariance_report_time).nanoseconds / 1e9
            if elapsed < self.report_throttle_sec:
                return

        if self.current_covariance > self.covariance_error_threshold:
            self.report_fault(
                fault_code='LOCALIZATION_UNCERTAINTY',
                severity=SEVERITY_ERROR,
                description=f'AMCL localization uncertainty very high: {self.current_covariance:.3f}',
                event_type=EVENT_FAILED
            )
            self.last_covariance_report_time = now
            self.get_logger().warn(f'High localization uncertainty: {self.current_covariance:.3f}')

        elif self.current_covariance > self.covariance_warn_threshold:
            self.report_fault(
                fault_code='LOCALIZATION_UNCERTAINTY',
                severity=SEVERITY_WARN,
                description=f'AMCL localization uncertainty elevated: {self.current_covariance:.3f}',
                event_type=EVENT_FAILED
            )
            self.last_covariance_report_time = now
        else:
            # Send PASSED to clear previous warnings
            if self.pending_heal_reports.get('LOCALIZATION_UNCERTAINTY'):
                self.report_fault(
                    fault_code='LOCALIZATION_UNCERTAINTY',
                    severity=SEVERITY_INFO,
                    description='AMCL localization normal',
                    event_type=EVENT_PASSED
                )
                self.last_covariance_report_time = now

    def odom_callback(self, msg: Odometry):
        """Track robot position for progress monitoring."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_position is not None:
            last_x, last_y = self.last_position
            distance = math.sqrt((x - last_x)**2 + (y - last_y)**2)

            if distance > self.min_progress_distance:
                now = self.get_clock().now()
                self.last_progress_time = now
                # Clear no-progress fault if robot is moving. Odometry arrives far
                # faster than the fault manager needs, and the burst has to be
                # spread out, so this shares the FAILED side's throttle: one report
                # per code per report_throttle_sec, whichever direction it goes.
                if self.pending_heal_reports.get('NAVIGATION_NO_PROGRESS') and self._may_report_no_progress(now):
                    self.report_fault(
                        fault_code='NAVIGATION_NO_PROGRESS',
                        severity=SEVERITY_INFO,
                        description='Robot making progress',
                        event_type=EVENT_PASSED
                    )
                    self.last_no_progress_report_time = now

        self.last_position = (x, y)

    def _may_report_no_progress(self, now: Time) -> bool:
        """True when the no-progress throttle window has elapsed.

        Shared by the FAILED and PASSED sides so the code is reported at most
        once per report_throttle_sec regardless of direction.
        """
        if self.last_no_progress_report_time is None:
            return True
        elapsed = (now - self.last_no_progress_report_time).nanoseconds / 1e9
        return elapsed > self.report_throttle_sec

    def check_timer_callback(self):
        """Periodic check for no-progress condition."""
        if not self.has_active_goal:
            return

        if self.last_progress_time is None:
            return

        now = self.get_clock().now()
        time_since_progress = (now - self.last_progress_time).nanoseconds / 1e9

        if time_since_progress > self.no_progress_timeout_sec:
            if self._may_report_no_progress(now):
                self.report_fault(
                    fault_code='NAVIGATION_NO_PROGRESS',
                    severity=SEVERITY_WARN,
                    description=f'No navigation progress for {time_since_progress:.1f}s',
                    event_type=EVENT_FAILED
                )
                self.last_no_progress_report_time = now
                self.get_logger().warn(f'No navigation progress for {time_since_progress:.1f}s')

    def report_fault(self, fault_code: str, severity: int, description: str, event_type: int,
                     source_suffix: str = ''):
        """Report a fault to FaultManager via service call.

        source_suffix separates reporters that behave differently. The
        no-progress check repeats while the condition holds, so a count-based
        debounce can filter it. The goal-status faults fire once on a status
        change, so any threshold past the first event would hide them for good.
        Reporting them under their own source lets the debounce configuration
        give each the threshold that suits it.
        """
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = severity
        request.description = description
        request.source_id = self.get_fully_qualified_name() + source_suffix

        # A FAILED event (re-)arms the healing burst; each PASSED spends one of it.
        if event_type == EVENT_FAILED:
            self.pending_heal_reports[fault_code] = HEAL_PASSED_REPEATS
        else:
            remaining = self.pending_heal_reports.get(fault_code, 0) - 1
            if remaining > 0:
                self.pending_heal_reports[fault_code] = remaining
            else:
                self.pending_heal_reports.pop(fault_code, None)

        # Async service call
        future = self.fault_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_fault_response(f, fault_code, event_type)
        )

    def _handle_fault_response(self, future, fault_code: str, event_type: int):
        """Handle response from fault_manager service."""
        try:
            response = future.result()
            if not response.accepted:
                self.get_logger().warn(f'Fault report rejected: {fault_code}')
        except Exception as e:
            self.get_logger().error(f'Failed to report fault {fault_code}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('AnomalyDetectorNode interrupted by user (KeyboardInterrupt), shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
