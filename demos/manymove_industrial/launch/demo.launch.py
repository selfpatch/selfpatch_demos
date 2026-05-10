"""manymove_industrial demo launch.

Reuses the upstream manymove xarm7_movegroup_fake_cpp_trees launch verbatim
so the BT pipeline behaves exactly as in the manymove project's own demos,
and adds the medkit fault_manager + gateway against the SOVD manifest.

The bt_client_xarm7 binary inside the manymove fork is the one that
actually emits MANYMOVE_* fault codes through ros2_medkit_fault_reporter
when the BT runs into collision / retry / IO failures.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    log_level = LaunchConfiguration("log_level")

    demo_pkg = FindPackageShare("manymove_industrial")
    bringup_pkg = FindPackageShare("manymove_bringup")

    manifest_path = PathJoinSubstitution([demo_pkg, "config", "manymove_industrial_manifest.yaml"])
    medkit_params = PathJoinSubstitution([demo_pkg, "config", "medkit_params.yaml"])

    # Upstream manymove xArm7 movegroup + BT pipeline (fake hardware).
    manymove_xarm7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            bringup_pkg, "/launch/xarm7_movegroup_fake_cpp_trees.launch.py",
        ]),
        launch_arguments={"log_level": log_level}.items(),
    )

    fault_manager = Node(
        package="ros2_medkit_fault_manager",
        executable="fault_manager_node",
        name="fault_manager",
        output="screen",
        parameters=[medkit_params],
    )

    gateway = Node(
        package="ros2_medkit_gateway",
        executable="gateway_node",
        name="ros2_medkit_gateway",
        namespace="diagnostics",
        output="screen",
        parameters=[medkit_params, {"discovery.manifest_path": manifest_path}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Suppress GUI tools (rviz/groot/HMI) for CI runs.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level forwarded to the manymove BT client.",
        ),
        manymove_xarm7,
        fault_manager,
        gateway,
    ])
