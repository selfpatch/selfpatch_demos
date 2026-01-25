"""Launch Sensor Diagnostics Demo with ros2_medkit gateway.

Lightweight demo without Gazebo - pure sensor simulation with fault injection.

Demonstrates two fault reporting paths:
1. Legacy path: Sensors → /diagnostics topic → diagnostic_bridge → fault_manager
   - Used by: LiDAR, Camera
   - Standard ROS 2 diagnostics pattern

2. Modern path: Sensors → anomaly_detector → ReportFault service → fault_manager
   - Used by: IMU, GPS
   - Direct ros2_medkit fault reporting

Namespace structure:
  /sensors - Simulated sensor nodes (lidar, imu, gps, camera)
  /processing - Anomaly detector
  /diagnostics - ros2_medkit gateway
  /bridge - Diagnostic bridge (legacy path)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory("sensor_diagnostics_demo")

    # Config file paths
    medkit_params_file = os.path.join(pkg_dir, "config", "medkit_params.yaml")
    sensor_params_file = os.path.join(pkg_dir, "config", "sensor_params.yaml")
    manifest_file = os.path.join(pkg_dir, "config", "sensor_manifest.yaml")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time (set to true if using with Gazebo)",
            ),
            # ===== Sensor Nodes (under /sensors namespace) =====
            # Legacy path sensors: publish DiagnosticArray to /diagnostics
            Node(
                package="sensor_diagnostics_demo",
                executable="lidar_sim_node",
                name="lidar_sim",
                namespace="sensors",
                output="screen",
                parameters=[sensor_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="sensor_diagnostics_demo",
                executable="camera_sim_node",
                name="camera_sim",
                namespace="sensors",
                output="screen",
                parameters=[sensor_params_file, {"use_sim_time": use_sim_time}],
            ),
            # Modern path sensors: monitored by anomaly_detector → ReportFault
            Node(
                package="sensor_diagnostics_demo",
                executable="imu_sim_node",
                name="imu_sim",
                namespace="sensors",
                output="screen",
                parameters=[sensor_params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="sensor_diagnostics_demo",
                executable="gps_sim_node",
                name="gps_sim",
                namespace="sensors",
                output="screen",
                parameters=[sensor_params_file, {"use_sim_time": use_sim_time}],
            ),
            # ===== Processing Nodes (under /processing namespace) =====
            # Modern path: anomaly_detector monitors IMU/GPS and calls ReportFault
            Node(
                package="sensor_diagnostics_demo",
                executable="anomaly_detector_node",
                name="anomaly_detector",
                namespace="processing",
                output="screen",
                parameters=[sensor_params_file, {"use_sim_time": use_sim_time}],
            ),
            # ===== Diagnostic Bridge (Legacy path) =====
            # Bridges /diagnostics topic (DiagnosticArray) → fault_manager
            # Handles faults from: LiDAR, Camera
            Node(
                package="ros2_medkit_diagnostic_bridge",
                executable="diagnostic_bridge_node",
                name="diagnostic_bridge",
                namespace="bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "diagnostics_topic": "/diagnostics",
                        "auto_generate_codes": True,
                    }
                ],
            ),
            # ===== ros2_medkit Gateway (under /diagnostics namespace) =====
            Node(
                package="ros2_medkit_gateway",
                executable="gateway_node",
                name="ros2_medkit_gateway",
                namespace="diagnostics",
                output="screen",
                parameters=[
                    medkit_params_file,
                    {"use_sim_time": use_sim_time},
                    {"manifest.path": manifest_file},
                ],
            ),
            # ===== Fault Manager (at root namespace) =====
            # Services at /fault_manager/* (e.g., /fault_manager/report_fault)
            # Both paths report here: diagnostic_bridge (legacy) and anomaly_detector (modern)
            Node(
                package="ros2_medkit_fault_manager",
                executable="fault_manager_node",
                name="fault_manager",
                namespace="",  # Root namespace so services are at /fault_manager/*
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
