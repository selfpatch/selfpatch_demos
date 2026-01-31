"""Launch TurtleBot3 simulation with Nav2 and ros2_medkit gateway for discovery demo.

This launch file demonstrates ros2_medkit's hierarchical discovery by:
  - Running TurtleBot3 + Nav2 (in root namespace)
  - Adding ros2_medkit gateway under /diagnostics namespace
  - Running fault_manager for fault aggregation
  - Running diagnostic_bridge for legacy /diagnostics support
  - Showing how nodes are organized into Areas based on namespaces
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")
    ros_gz_sim_dir = get_package_share_directory("ros_gz_sim")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    demo_pkg_dir = get_package_share_directory("turtlebot3_medkit_demo")

    # Path to config files from installed package
    medkit_params_file = os.path.join(demo_pkg_dir, "config", "medkit_params.yaml")
    nav2_params_file = os.path.join(demo_pkg_dir, "config", "nav2_params.yaml")
    map_file = os.path.join(demo_pkg_dir, "config", "turtlebot3_world.yaml")
    manifest_file = os.path.join(demo_pkg_dir, "config", "turtlebot3_manifest.yaml")

    # Gazebo world file
    world_file = os.path.join(turtlebot3_gazebo_dir, "worlds", "turtlebot3_world.world")

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    headless = LaunchConfiguration("headless", default="False")
    x_pose = LaunchConfiguration("x_pose", default="-2.0")
    y_pose = LaunchConfiguration("y_pose", default="-0.5")

    # Gazebo model path
    set_gz_model_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(turtlebot3_gazebo_dir, "models"),
    )

    return LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="False",
                description="Run Gazebo without GUI (headless=True for Docker/CI, False for local GUI)",
            ),
            DeclareLaunchArgument(
                "x_pose",
                default_value="-2.0",
                description="Robot initial X position",
            ),
            DeclareLaunchArgument(
                "y_pose",
                default_value="-0.5",
                description="Robot initial Y position",
            ),
            # Set TurtleBot3 model (can be overridden by environment variable)
            SetEnvironmentVariable(
                name="TURTLEBOT3_MODEL",
                value=os.environ.get("TURTLEBOT3_MODEL", "burger"),
            ),
            set_gz_model_path,
            # === HEADLESS MODE: Gazebo server only ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ros_gz_sim_dir, "launch", "gz_sim.launch.py")
                ),
                launch_arguments={
                    "gz_args": ["-r", "-s", "-v2", world_file],
                    "on_exit_shutdown": "true",
                }.items(),
                condition=IfCondition(headless),
            ),
            # === GUI MODE: Full TurtleBot3 world (server + client) ===
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_gazebo_dir, "launch", "turtlebot3_world.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=UnlessCondition(headless),
            ),
            # Spawn TurtleBot3 robot (headless mode only - GUI mode already includes spawn)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_gazebo_dir, "launch", "spawn_turtlebot3.launch.py")
                ),
                launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
                condition=IfCondition(headless),
            ),
            # Robot state publisher (headless mode only - GUI mode includes it)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_gazebo_dir, "launch", "robot_state_publisher.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=IfCondition(headless),
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
            # Launch ros2_medkit fault_manager in root namespace
            # Aggregates faults from all nodes via ReportFault service
            Node(
                package="ros2_medkit_fault_manager",
                executable="fault_manager_node",
                name="fault_manager",
                namespace="",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # Launch diagnostic_bridge under /bridge namespace
            # Converts legacy /diagnostics topic to faults
            Node(
                package="ros2_medkit_diagnostic_bridge",
                executable="diagnostic_bridge_node",
                name="diagnostic_bridge",
                namespace="bridge",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # Launch ros2_medkit gateway under /diagnostics namespace
            # This demonstrates namespace-based Area organization in discovery
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
            # Launch anomaly detector to monitor navigation and publish diagnostics
            Node(
                package="turtlebot3_medkit_demo",
                executable="anomaly_detector.py",
                name="anomaly_detector",
                namespace="bridge",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"covariance_warn_threshold": 0.3},
                    {"covariance_error_threshold": 1.0},
                    {"no_progress_timeout_sec": 20.0},
                ],
            ),
        ]
    )
