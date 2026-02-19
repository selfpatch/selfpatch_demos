#!/usr/bin/env python3
"""Fault storm injector for ros2_medkit debounce demo.

Fires 9 noise faults (1 report each) + 1 real problem (5 sustained reports),
interleaved so the real fault is buried in the noise.

  Storm (threshold=0):   all 10 CONFIRMED - wall of warnings, real issue invisible
  Debounce (threshold=-3): 9 PREFAILED + 1 CONFIRMED - real issue pops out

Requires ros2_medkit built in your workspace.

Usage (standalone):
    python3 fault_storm.py

Usage (Docker demo):
    docker exec turtlebot3_medkit_demo bash -c \
        "source /opt/ros/jazzy/setup.bash && \
         source /root/demo_ws/install/setup.bash && \
         python3 /root/demo_ws/src/turtlebot3_medkit_demo/scripts/fault_storm.py"
"""

import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.srv import ReportFault
import time


# === THE REAL PROBLEM (sustained, confirms even with debounce) ===
REAL_FAULT = {
    "fault_code": "MOTOR_OVERHEAT",
    "severity": 1,  # WARN (same as noise — invisible in storm mode)
    "description": "Drive motor temperature 92C (limit: 80C)",
    "source_id": "/drive/thermal_monitor",
    "burst": 5,
}

# === NOISE: 9 faults across 3 categories → wall of similar alerts ===
NOISE_FAULTS = [
    # Sensor noise cluster (4 faults)
    {
        "fault_code": "SENSOR_NOISE_LIDAR",
        "severity": 1,
        "description": "LiDAR std dev: 0.12m (limit: 0.03m)",
        "source_id": "/sensors/lidar",
    },
    {
        "fault_code": "SENSOR_NOISE_IMU",
        "severity": 1,
        "description": "IMU gyro drift: 1.8 deg/s (limit: 0.5 deg/s)",
        "source_id": "/sensors/imu",
    },
    {
        "fault_code": "SENSOR_NOISE_CAMERA",
        "severity": 1,
        "description": "Depth camera: 3 frames dropped in 1s",
        "source_id": "/sensors/camera",
    },
    {
        "fault_code": "SENSOR_NOISE_ODOM",
        "severity": 1,
        "description": "Wheel odometry jitter: 0.12 m/s (limit: 0.05 m/s)",
        "source_id": "/sensors/odom",
    },
    # Nav timeout cluster (4 faults)
    {
        "fault_code": "NAV_TIMEOUT_CONTROLLER",
        "severity": 1,
        "description": "Controller loop: 62ms (limit: 50ms)",
        "source_id": "/nav/controller",
    },
    {
        "fault_code": "NAV_TIMEOUT_PLANNER",
        "severity": 1,
        "description": "Planner response: 320ms (limit: 200ms)",
        "source_id": "/nav/planner",
    },
    {
        "fault_code": "NAV_TIMEOUT_COSTMAP",
        "severity": 1,
        "description": "Costmap update: 250ms (limit: 100ms)",
        "source_id": "/nav/costmap",
    },
    {
        "fault_code": "NAV_TIMEOUT_TF",
        "severity": 1,
        "description": "TF lookup: 85ms (limit: 50ms)",
        "source_id": "/nav/tf_monitor",
    },
    # Comms cluster (1 fault)
    {
        "fault_code": "COMM_LATENCY_WIFI",
        "severity": 1,
        "description": "WiFi round-trip: 230ms (limit: 100ms)",
        "source_id": "/network/wifi_monitor",
    },
]


class FaultStormNode(Node):
    def __init__(self):
        super().__init__("fault_storm_injector")
        self.client = self.create_client(
            ReportFault, "/fault_manager/report_fault"
        )
        self.get_logger().info("Waiting for fault_manager...")
        self.client.wait_for_service(timeout_sec=5.0)
        self.get_logger().info("Connected")

    def fire(self, fault_code, severity, description, source_id):
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = severity
        req.description = description
        req.source_id = source_id
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        return future.result() and future.result().accepted

    def run_storm(self):
        self.get_logger().info("=== STORM ===")
        n = 0

        # Interleave: noise, noise, REAL, noise, noise, REAL, ...
        # MOTOR_OVERHEAT buried among SENSOR_NOISE, NAV_TIMEOUT, and COMM faults
        real_sent = 0
        noise_idx = 0
        sequence = [
            "noise", "noise", "REAL",       # lidar, imu, MOTOR #1
            "noise", "noise", "REAL",       # camera, odom, MOTOR #2
            "noise", "noise", "REAL",       # controller, planner, MOTOR #3
            "noise", "noise", "noise",      # costmap, tf, wifi
            "REAL", "REAL",                 # MOTOR #4, #5
        ]

        for step in sequence:
            if step == "REAL" and real_sent < REAL_FAULT["burst"]:
                f = REAL_FAULT
                real_sent += 1
                label = f"MOTOR_OVERHEAT #{real_sent}"
            elif step == "noise" and noise_idx < len(NOISE_FAULTS):
                f = NOISE_FAULTS[noise_idx]
                noise_idx += 1
                label = f["fault_code"]
            else:
                continue

            ok = self.fire(f["fault_code"], f["severity"],
                           f["description"], f["source_id"])
            n += 1
            self.get_logger().info(
                f"  [{n:2d}] {label} -> {'OK' if ok else 'FAIL'}")
            time.sleep(0.4)

        self.get_logger().info(f"=== DONE: {n} events ===")


def main():
    rclpy.init()
    node = FaultStormNode()
    node.run_storm()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
