"""Launch TurtleBot3 simulation with Nav2 and ros2_medkit gateway for discovery demo.

This launch file demonstrates ros2_medkit's hierarchical discovery by:
  - Running TurtleBot3 + Nav2 (in root namespace)
  - Adding ros2_medkit gateway under /diagnostics namespace
  - Showing how nodes are organized into Areas based on namespaces
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    demo_pkg_dir = get_package_share_directory("turtlebot3_medkit_demo")

    # Path to config files from installed package
    medkit_params_file = os.path.join(demo_pkg_dir, "config", "medkit_params.yaml")
    nav2_params_file = os.path.join(demo_pkg_dir, "config", "nav2_params.yaml")
    map_file = os.path.join(demo_pkg_dir, "config", "turtlebot3_world.yaml")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            # Set TurtleBot3 model (can be overridden by environment variable)
            SetEnvironmentVariable(
                name="TURTLEBOT3_MODEL",
                value=os.environ.get("TURTLEBOT3_MODEL", "burger"),
            ),
            # Launch TurtleBot3 Gazebo simulation (turtlebot3_world)
            # Runs in root namespace to publish standard topics (/scan, /odom, /cmd_vel)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        turtlebot3_gazebo_dir, "launch", "turtlebot3_world.launch.py"
                    )
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
            ),
            # Launch Nav2 navigation stack
            # Runs in root namespace to subscribe to robot topics
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "params_file": nav2_params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": "True",
                }.items(),
            ),
            # Launch ros2_medkit gateway under /diagnostics namespace
            # This demonstrates namespace-based Area organization in discovery
            Node(
                package="ros2_medkit_gateway",
                executable="gateway_node",
                name="ros2_medkit_gateway",
                namespace="diagnostics",
                output="screen",
                parameters=[medkit_params_file, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
