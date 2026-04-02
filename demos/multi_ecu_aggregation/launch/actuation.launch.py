# Copyright 2026 selfpatch
# Licensed under the Apache License, Version 2.0

"""Launch Actuation ECU nodes with ros2_medkit gateway (mDNS announce enabled).

Actuation ECU runs as a standalone peer with mDNS announcement so the
perception aggregator can discover it automatically. A domain_bridge node
bridges planning commands from domain 20 to domain 30.

Nodes launched:
  /actuation/motor_controller   - Motor torque control
  /actuation/joint_driver       - Joint position driver
  /actuation/gripper_controller - Gripper open/close control
  /diagnostics/ros2_medkit_gateway - SOVD gateway (mDNS announce enabled)
  /fault_manager                - Fault aggregation and storage
  /bridge/diagnostic_bridge     - Bridges /diagnostics topic to fault_manager
  actuation_bridge              - Domain bridge for cross-ECU topic relay
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
    params_file = os.path.join(pkg_dir, 'config', 'actuation_params.yaml')
    manifest_file = os.path.join(
        pkg_dir, 'config', 'actuation_manifest.yaml')
    bridge_config_path = os.path.join(
        pkg_dir, 'config', 'domain_bridge', 'actuation_bridge.yaml')

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
        # ===== Actuation Nodes (under /actuation namespace) =====
        Node(
            package='multi_ecu_demo',
            executable='motor_controller',
            name='motor_controller',
            namespace='actuation',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='multi_ecu_demo',
            executable='joint_driver',
            name='joint_driver',
            namespace='actuation',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='multi_ecu_demo',
            executable='gripper_controller',
            name='gripper_controller',
            namespace='actuation',
            output='screen',
            parameters=[params_file],
        ),

        # ===== Domain Bridge (cross-ECU topic relay) =====
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='actuation_bridge',
            arguments=[bridge_config_path],
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

        # ===== ros2_medkit Gateway (mDNS announce enabled) =====
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
