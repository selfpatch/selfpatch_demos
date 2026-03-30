# Copyright 2026 selfpatch
# Licensed under the Apache License, Version 2.0

"""Launch Planning ECU nodes with ros2_medkit gateway (no aggregation).

Planning ECU runs as a standalone peer - it does not aggregate other ECUs.
A domain_bridge node bridges perception detections from domain 10 to domain 20.

Nodes launched:
  /planning/path_planner        - Path planning from detection data
  /planning/behavior_planner    - Behavior decision-making
  /planning/task_scheduler      - Task queue management
  /diagnostics/ros2_medkit_gateway - SOVD gateway (aggregation disabled)
  /fault_manager                - Fault aggregation and storage
  /bridge/diagnostic_bridge     - Bridges /diagnostics topic to fault_manager
  planning_bridge               - Domain bridge for cross-ECU topic relay
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
    params_file = os.path.join(pkg_dir, 'config', 'planning_params.yaml')
    manifest_file = os.path.join(
        pkg_dir, 'config', 'planning_manifest.yaml')
    bridge_config_path = os.path.join(
        pkg_dir, 'config', 'domain_bridge', 'planning_bridge.yaml')

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
        # ===== Planning Nodes (under /planning namespace) =====
        Node(
            package='multi_ecu_demo',
            executable='path_planner',
            name='path_planner',
            namespace='planning',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='multi_ecu_demo',
            executable='behavior_planner',
            name='behavior_planner',
            namespace='planning',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='multi_ecu_demo',
            executable='task_scheduler',
            name='task_scheduler',
            namespace='planning',
            output='screen',
            parameters=[params_file],
        ),

        # ===== Domain Bridge (cross-ECU topic relay) =====
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='planning_bridge',
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

        # ===== ros2_medkit Gateway (no aggregation) =====
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
