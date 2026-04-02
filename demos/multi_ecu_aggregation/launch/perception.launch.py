# Copyright 2026 selfpatch
# Licensed under the Apache License, Version 2.0

"""Launch Perception ECU nodes with ros2_medkit gateway (aggregation ON).

Perception ECU is the primary aggregator - it pulls diagnostics data from
planning and actuation ECUs via the gateway aggregation feature.

Nodes launched:
  /perception/lidar_driver      - LiDAR sensor driver
  /perception/camera_driver     - Camera sensor driver
  /perception/point_cloud_filter - Point cloud preprocessing
  /perception/object_detector   - Object detection pipeline
  /diagnostics/ros2_medkit_gateway - SOVD gateway (aggregation enabled)
  /fault_manager                - Fault aggregation and storage
  /bridge/diagnostic_bridge     - Bridges /diagnostics topic to fault_manager
"""

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchDescription
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
    pkg_dir = get_package_share_directory('multi_ecu_demo')

    # Config file paths
    params_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')
    manifest_file = os.path.join(
        pkg_dir, 'config', 'perception_manifest.yaml')

    # Resolve optional plugin paths
    graph_provider_path = _resolve_plugin_path(
        'ros2_medkit_graph_provider', 'ros2_medkit_graph_provider_plugin')

    plugin_overrides = {}
    active_plugins = []
    if graph_provider_path:
        active_plugins.append('graph_provider')
        plugin_overrides['plugins.graph_provider.path'] = graph_provider_path
    plugin_overrides['plugins'] = active_plugins

    return LaunchDescription([
        # ===== Perception Nodes (under /perception namespace) =====
        Node(
            package='multi_ecu_demo',
            executable='lidar_driver',
            name='lidar_driver',
            namespace='perception',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='multi_ecu_demo',
            executable='camera_driver',
            name='camera_driver',
            namespace='perception',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='multi_ecu_demo',
            executable='point_cloud_filter',
            name='point_cloud_filter',
            namespace='perception',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='multi_ecu_demo',
            executable='object_detector',
            name='object_detector',
            namespace='perception',
            output='screen',
            parameters=[params_file],
        ),

        # ===== Diagnostic Bridge (Legacy path) =====
        Node(
            package='ros2_medkit_diagnostic_bridge',
            executable='diagnostic_bridge_node',
            name='diagnostic_bridge',
            namespace='bridge',
            output='screen',
            parameters=[{
                'diagnostics_topic': '/diagnostics',
                'auto_generate_codes': True,
            }],
        ),

        # ===== ros2_medkit Gateway (aggregation enabled) =====
        Node(
            package='ros2_medkit_gateway',
            executable='gateway_node',
            name='ros2_medkit_gateway',
            namespace='diagnostics',
            output='screen',
            parameters=[
                params_file,
                {'discovery.manifest_path': manifest_file},
                plugin_overrides,
            ],
        ),

        # ===== Fault Manager (root namespace) =====
        Node(
            package='ros2_medkit_fault_manager',
            executable='fault_manager_node',
            name='fault_manager',
            namespace='',
            output='screen',
            parameters=[params_file],
        ),
    ])
