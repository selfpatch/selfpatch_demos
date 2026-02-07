"""Launch Panda robot with MoveIt 2 and ros2_medkit for diagnostics demo.

This launch file starts:
  - Panda 7-DOF arm with fake hardware (mock controllers, no physics sim)
  - MoveIt 2 move_group for motion planning (via included demo launch)
  - RViz for visualization (non-headless mode only)
  - Continuous pick-and-place task loop
  - ros2_medkit gateway, fault_manager, diagnostic_bridge
  - Manipulation anomaly monitor
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    demo_pkg_dir = get_package_share_directory("moveit_medkit_demo")
    panda_moveit_config_dir = get_package_share_directory(
        "moveit_resources_panda_moveit_config"
    )

    # Config file paths
    medkit_params_file = os.path.join(demo_pkg_dir, "config", "medkit_params.yaml")
    manifest_file = os.path.join(demo_pkg_dir, "config", "panda_manifest.yaml")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    headless = LaunchConfiguration("headless", default="False")

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="False",
                description="Use simulation clock if true (False for fake hardware)",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="False",
                description="Run without GUI (True for Docker/CI)",
            ),
            # Suppress GUI environment when headless
            SetEnvironmentVariable(
                name="QT_QPA_PLATFORM",
                value="offscreen",
                condition=IfCondition(headless),
            ),
            # Include MoveIt 2 Panda demo launch (with GUI)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        panda_moveit_config_dir, "launch", "demo.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_rviz": "true",
                }.items(),
                condition=UnlessCondition(headless),
            ),
            # Include MoveIt 2 Panda demo launch (headless — no RViz)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        panda_moveit_config_dir, "launch", "demo.launch.py"
                    )
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "use_rviz": "false",
                }.items(),
                condition=IfCondition(headless),
            ),
            # === ros2_medkit stack ===
            # Fault manager — root namespace
            Node(
                package="ros2_medkit_fault_manager",
                executable="fault_manager_node",
                name="fault_manager",
                namespace="",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # Diagnostic bridge — /bridge namespace
            # auto_generate_codes=False prevents noisy controller_manager
            # periodicity diagnostics from being reported as faults.
            # The manipulation_monitor handles all demo-relevant faults directly.
            Node(
                package="ros2_medkit_diagnostic_bridge",
                executable="diagnostic_bridge_node",
                name="diagnostic_bridge",
                namespace="bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "auto_generate_codes": False,
                    }
                ],
            ),
            # Gateway — /diagnostics namespace
            Node(
                package="ros2_medkit_gateway",
                executable="gateway_node",
                name="ros2_medkit_gateway",
                namespace="diagnostics",
                output="screen",
                parameters=[
                    medkit_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "manifest_path": manifest_file,
                    },
                ],
            ),
            # === Demo scripts ===
            # Pick-and-place loop
            Node(
                package="moveit_medkit_demo",
                executable="pick_place_loop.py",
                name="pick_place_loop",
                namespace="",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # Manipulation monitor (anomaly detector)
            Node(
                package="moveit_medkit_demo",
                executable="manipulation_monitor.py",
                name="manipulation_monitor",
                namespace="bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
