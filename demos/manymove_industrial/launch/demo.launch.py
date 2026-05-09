"""manymove_industrial demo launch.

Brings up the medkit fault_manager and gateway against the SOVD manifest. The
manymove BT client and move_group are intentionally NOT started here yet
(blocked on xArm packages being available on jazzy or a Panda variant of the
demo). Once that lands, drop the corresponding Node() entries in the
LaunchDescription below.

Faults can already be exercised against this stack via the inject scripts
under container_scripts/manymove-planning/, which talk to
/fault_manager/report_fault directly.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")

    pkg_share = FindPackageShare("manymove_industrial")
    manifest_path = PathJoinSubstitution([
        pkg_share, "config", "manymove_industrial_manifest.yaml",
    ])
    medkit_params = PathJoinSubstitution([
        pkg_share, "config", "medkit_params.yaml",
    ])

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
        output="screen",
        parameters=[
            medkit_params,
            {"discovery.manifest_path": manifest_path},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Suppress GUI / visualization (no-op until move_group is wired up).",
        ),
        fault_manager,
        gateway,
    ])
