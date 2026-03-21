"""Launch Sensor Diagnostics Demo with ros2_medkit gateway.

Lightweight demo without Gazebo - pure sensor simulation with fault injection.

Demonstrates two fault reporting paths:
1. Legacy path: Sensors -> /diagnostics topic -> diagnostic_bridge -> fault_manager
   - Used by: LiDAR, Camera
   - Standard ROS 2 diagnostics pattern

2. Modern path: Sensors -> anomaly_detector -> ReportFault service -> fault_manager
   - Used by: IMU, GPS
   - Direct ros2_medkit fault reporting

Beacon modes (set via BEACON_MODE env var):
  none  - No beacon plugins (default)
  topic - Topic beacon: sensor nodes push MedkitDiscoveryHint messages
  param - Parameter beacon: gateway polls sensor node parameters

Namespace structure:
  /sensors - Simulated sensor nodes (lidar, imu, gps, camera)
  /processing - Anomaly detector
  /diagnostics - ros2_medkit gateway
  /bridge - Diagnostic bridge (legacy path)
"""

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_plugin_path(package_name, lib_name):
    """Resolve a gateway plugin .so path, returning empty string if not found."""
    try:
        prefix = get_package_prefix(package_name)
        path = os.path.join(prefix, 'lib', package_name, f'lib{lib_name}.so')
        if os.path.isfile(path):
            return path
    except PackageNotFoundError:
        pass
    return ''


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory("sensor_diagnostics_demo")

    # Config file paths
    medkit_params_file = os.path.join(pkg_dir, "config", "medkit_params.yaml")
    sensor_params_file = os.path.join(pkg_dir, "config", "sensor_params.yaml")
    manifest_file = os.path.join(pkg_dir, "config", "sensor_manifest.yaml")

    # Beacon mode from environment (controls both plugin loading and node behavior)
    beacon_mode = os.environ.get('BEACON_MODE', 'none')
    valid_beacon_modes = ('none', 'topic', 'param')
    if beacon_mode not in valid_beacon_modes:
        import sys
        print(
            f"WARNING: Invalid BEACON_MODE='{beacon_mode}'. "
            f"Valid values: {', '.join(valid_beacon_modes)}. "
            "Falling back to 'none'.",
            file=sys.stderr,
        )
        beacon_mode = 'none'

    # Resolve plugin paths
    graph_provider_path = _resolve_plugin_path(
        'ros2_medkit_graph_provider', 'ros2_medkit_graph_provider_plugin')
    procfs_plugin_path = _resolve_plugin_path(
        'ros2_medkit_linux_introspection', 'procfs_introspection')

    # Build plugin overrides - only include plugins that were found
    plugin_overrides = {}
    active_plugins = []
    if graph_provider_path:
        active_plugins.append('graph_provider')
        plugin_overrides['plugins.graph_provider.path'] = graph_provider_path
    if procfs_plugin_path:
        active_plugins.append('procfs_introspection')
        plugin_overrides['plugins.procfs_introspection.path'] = procfs_plugin_path

    # Beacon plugin (mutually exclusive - only one beacon type at a time)
    if beacon_mode == 'topic':
        topic_beacon_path = _resolve_plugin_path(
            'ros2_medkit_topic_beacon', 'topic_beacon_plugin')
        if topic_beacon_path:
            active_plugins.append('topic_beacon')
            plugin_overrides['plugins.topic_beacon.path'] = topic_beacon_path
            plugin_overrides['plugins.topic_beacon.topic'] = \
                '/ros2_medkit/discovery'
            plugin_overrides['plugins.topic_beacon.beacon_ttl_sec'] = 10.0
        else:
            import sys
            print(
                "WARNING: BEACON_MODE=topic but topic_beacon plugin not "
                "found. Falling back to none.",
                file=sys.stderr,
            )
            beacon_mode = 'none'
    elif beacon_mode == 'param':
        param_beacon_path = _resolve_plugin_path(
            'ros2_medkit_param_beacon', 'param_beacon_plugin')
        if param_beacon_path:
            active_plugins.append('parameter_beacon')
            plugin_overrides['plugins.parameter_beacon.path'] = \
                param_beacon_path
            plugin_overrides['plugins.parameter_beacon.poll_interval_sec'] = \
                5.0
        else:
            import sys
            print(
                "WARNING: BEACON_MODE=param but param_beacon plugin not "
                "found. Falling back to none.",
                file=sys.stderr,
            )
            beacon_mode = 'none'

    plugin_overrides['plugins'] = active_plugins

    # Sensor node beacon parameter (passed to all sensor nodes)
    beacon_params = {"beacon_mode": beacon_mode}

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time (set to true if using "
                "with Gazebo)",
            ),
            # ===== Sensor Nodes (under /sensors namespace) =====
            # Legacy path sensors: publish DiagnosticArray to /diagnostics
            Node(
                package="sensor_diagnostics_demo",
                executable="lidar_sim_node",
                name="lidar_sim",
                namespace="sensors",
                output="screen",
                parameters=[
                    sensor_params_file,
                    {"use_sim_time": use_sim_time},
                    beacon_params,
                ],
            ),
            Node(
                package="sensor_diagnostics_demo",
                executable="camera_sim_node",
                name="camera_sim",
                namespace="sensors",
                output="screen",
                parameters=[
                    sensor_params_file,
                    {"use_sim_time": use_sim_time},
                    beacon_params,
                ],
            ),
            # Modern path sensors: monitored by anomaly_detector -> ReportFault
            Node(
                package="sensor_diagnostics_demo",
                executable="imu_sim_node",
                name="imu_sim",
                namespace="sensors",
                output="screen",
                parameters=[
                    sensor_params_file,
                    {"use_sim_time": use_sim_time},
                    beacon_params,
                ],
            ),
            Node(
                package="sensor_diagnostics_demo",
                executable="gps_sim_node",
                name="gps_sim",
                namespace="sensors",
                output="screen",
                parameters=[
                    sensor_params_file,
                    {"use_sim_time": use_sim_time},
                    beacon_params,
                ],
            ),
            # ===== Processing Nodes (under /processing namespace) =====
            # Modern path: anomaly_detector monitors IMU/GPS, calls ReportFault
            Node(
                package="sensor_diagnostics_demo",
                executable="anomaly_detector_node",
                name="anomaly_detector",
                namespace="processing",
                output="screen",
                parameters=[
                    sensor_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            # ===== Diagnostic Bridge (Legacy path) =====
            # Bridges /diagnostics topic (DiagnosticArray) -> fault_manager
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
                    {"discovery.manifest_path": manifest_file},
                    plugin_overrides,
                ],
            ),
            # ===== Fault Manager (at root namespace) =====
            # Services at /fault_manager/* (e.g., /fault_manager/report_fault)
            # Both paths report here: diagnostic_bridge (legacy) and
            # anomaly_detector (modern)
            # Also handles snapshot and rosbag capture on fault confirmation
            Node(
                package="ros2_medkit_fault_manager",
                executable="fault_manager_node",
                name="fault_manager",
                namespace="",  # Root namespace: services at /fault_manager/*
                output="screen",
                parameters=[
                    medkit_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
