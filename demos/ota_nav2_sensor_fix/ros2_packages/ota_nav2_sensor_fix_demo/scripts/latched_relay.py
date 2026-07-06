#!/usr/bin/env python3
# Copyright 2026 bburda. Apache-2.0.
#
# latched_relay: keeps /robot_description (std_msgs/String) and /tf_static
# (tf2_msgs/TFMessage) "fresh" so a VOLATILE, best-effort-at-capture-time
# subscriber can actually receive them.
#
# Both topics are published exactly ONCE at startup - robot_state_publisher
# latches them with TRANSIENT_LOCAL durability. The fault_manager's
# ring-buffered rosbag snapshot subscribes to its include_topics with a
# VOLATILE QoS at fault-confirmation time (it has no way to know ahead of
# time which topics/QoS a given fault snapshot will need), so it never sees
# a late-joining latched sample. Every downloaded MCAP was therefore missing
# the robot URDF and the static TF tree (base_footprint -> base_link ->
# laser etc.), so Foxglove playback had no robot mesh and nowhere to place
# /scan relative to the robot.
#
# This node subscribes both topics ONCE with a TRANSIENT_LOCAL QoS (so it
# receives the one latched sample from robot_state_publisher), caches the
# last message of each, and re-publishes them periodically on a separate,
# VOLATILE QoS. Any volatile subscriber - including the fault_manager's
# rosbag capture - is then guaranteed to see a recent sample inside every
# ~7s capture window, not just the single instant at startup. Re-publishing
# the same static transforms / URDF repeatedly is idempotent for both TF2
# (static transforms have no expiry) and Foxglove.

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

_REPUBLISH_PERIOD_SEC = 0.5  # ~2 Hz


class LatchedRelay(Node):
    """Relay /robot_description + /tf_static from TRANSIENT_LOCAL (latched,
    published once) onto a periodic VOLATILE republish, so a volatile-QoS
    capture always sees a recent sample."""

    def __init__(self) -> None:
        super().__init__('latched_relay')
        # use_sim_time is auto-declared by every rclpy node and set to True via
        # the launch's `--ros-args -p use_sim_time:=True`, so this node follows
        # the sim clock. Do NOT re-declare it here - declare_parameter on an
        # already-declared parameter raises ParameterAlreadyDeclaredException and
        # kills the node on startup (which silently left the fault MCAP without
        # /robot_description + /tf_static, so playback showed no robot).

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._robot_description = None
        self._tf_static = None

        self._robot_description_pub = self.create_publisher(
            String, '/robot_description', volatile_qos)
        self._tf_static_pub = self.create_publisher(
            TFMessage, '/tf_static', volatile_qos)

        self.create_subscription(
            String, '/robot_description', self._on_robot_description, latched_qos)
        self.create_subscription(
            TFMessage, '/tf_static', self._on_tf_static, latched_qos)

        self.create_timer(_REPUBLISH_PERIOD_SEC, self._republish)

    def _on_robot_description(self, msg: String) -> None:
        self._robot_description = msg

    def _on_tf_static(self, msg: TFMessage) -> None:
        self._tf_static = msg

    def _republish(self) -> None:
        if self._robot_description is not None:
            self._robot_description_pub.publish(self._robot_description)
        if self._tf_static is not None:
            self._tf_static_pub.publish(self._tf_static)


def main() -> None:
    rclpy.init()
    node = LatchedRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
