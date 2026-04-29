# Copyright 2026 bburda
# Apache 2.0
#
# Single launch entry point for the OTA over SOVD nav2 sensor-fix demo.
#
# Brings up, in one container:
#   - headless Gazebo (gz-sim) with the TurtleBot3 world
#   - TurtleBot3 (burger) spawned in that world + robot_state_publisher
#   - the full Nav2 stack (bringup_launch.py) with the TB3 default map
#   - foxglove_bridge on :8765 so Foxglove Studio can render /tf, /scan, /map etc.
#   - ros2_medkit fault_manager (the gateway's /faults endpoint depends on it)
#   - broken_lidar + broken_lidar_legacy (the OTA plugin swaps/uninstalls these)
#   - the gateway with our ota_update_plugin loaded via gateway_config.yaml
#
# /scan-override strategy
# -----------------------
# The TB3 gz-sim simulation publishes its own LaserScan on /scan via the
# ros_gz_bridge. broken_lidar also publishes on /scan with a phantom obstacle
# at ray index 180. Two publishers on /scan would interleave - nav2 would see
# garbage. We solve this by wrapping spawn_turtlebot3 in a GroupAction with a
# SetRemap that pushes the simulator's /scan to /scan_sim. That way the gz
# bridge inside the spawn launch ends up emitting /scan_sim, not /scan, and
# broken_lidar (and later fixed_lidar) is the sole publisher on /scan that
# nav2 + foxglove see.

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
    PackageNotFoundError,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def _resolve_plugin_path(package_name, lib_name):
    """Resolve a gateway plugin .so path inside the colcon install tree."""
    try:
        prefix = get_package_prefix(package_name)
    except PackageNotFoundError:
        return ''
    candidates = [
        os.path.join(prefix, 'lib', package_name, f'lib{lib_name}.so'),
        os.path.join(prefix, 'lib', package_name, f'{lib_name}.so'),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    return ''


def generate_launch_description():
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    demo_pkg_dir = get_package_share_directory('ota_nav2_sensor_fix_demo')

    nav2_params_file = os.path.join(demo_pkg_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(demo_pkg_dir, 'config', 'turtlebot3_world.yaml')

    # OTA plugin shipped via the gateway image's /etc/ros2_medkit/gateway_config.yaml.
    # The plugin itself loads when gateway_node parses that params file - we just
    # point the gateway at it via --ros-args --params-file below.
    gateway_config_file = os.environ.get(
        'OTA_DEMO_GATEWAY_CONFIG',
        '/etc/ros2_medkit/gateway_config.yaml',
    )

    world_file = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    headless = LaunchConfiguration('headless', default='True')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    set_gz_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(turtlebot3_gazebo_dir, 'models'),
    )

    # Gazebo headless server: -r runs the world, -s is server-only (no GUI), -v2
    # is INFO logging. on_exit_shutdown ensures gz dying tears the launch down.
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': ['-r', '-s', '-v2', world_file],
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    # GUI mode is left in for parity with the sibling TB3 demo (someone running
    # this on a workstation can flip headless:=False); in containers we never
    # take this branch.
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'turtlebot3_world.launch.py'),
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=UnlessCondition(headless),
    )

    # Spawn the robot - SetRemap inside the GroupAction pushes the gazebo
    # bridge's /scan onto /scan_sim so broken_lidar owns /scan exclusively.
    spawn_robot = GroupAction(
        actions=[
            SetRemap(src='/scan', dst='/scan_sim'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py'),
                ),
                launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
            ),
        ],
        condition=IfCondition(headless),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py'),
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(headless),
    )

    # use_composition=False forces nav2 to launch each lifecycle node as its
    # own process instead of co-loading them into component_container_isolated.
    # The Jazzy apt build of nav2_msgs has an ABI mismatch against the
    # fastcdr 2.2.5 currently shipping with ros-jazzy-fastcdr (missing
    # eprosima::fastcdr::Cdr::serialize(unsigned int)) which immediately kills
    # the container at composition time. Per-node mode dodges that crash.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py'),
        ),
        launch_arguments={
            'map': map_file,
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'True',
            'use_composition': 'False',
        }.items(),
    )

    # healing_enabled lets EVENT_PASSED reports actually clear an active
    # fault instead of stalling at PREFAILED. We need this so fixed_lidar
    # (post-OTA) can clear the SCAN_PHANTOM_RETURN that broken_lidar
    # raised - otherwise the Faults Dashboard stays red forever after
    # the OTA swap. Threshold of 2 keeps things responsive while still
    # debouncing single accidental EVENT_PASSED reports.
    fault_manager = Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # `memory` storage so faults don't persist across container
            # restarts - the demo is supposed to start with a clean
            # Faults Dashboard every time, and SQLite kept the last
            # SCAN_PHANTOM_RETURN around forever.
            'storage_type': 'memory',
            # healing_enabled lets EVENT_PASSED reports actually heal an
            # active fault instead of stalling at PREFAILED (so
            # fixed_lidar can clear the fault that broken_lidar raised
            # via the OTA swap). healing_threshold=2 gives a small
            # debounce against single accidental clears.
            'healing_enabled': True,
            'healing_threshold': 2,
        }],
    )

    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[
            {'port': 8765},
            {'address': '0.0.0.0'},
            {'use_sim_time': use_sim_time},
        ],
    )

    broken_lidar_node = Node(
        package='broken_lidar',
        executable='broken_lidar_node',
        name='scan_sensor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    broken_lidar_legacy = Node(
        package='broken_lidar_legacy',
        executable='broken_lidar_legacy',
        name='broken_lidar_legacy',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Plugin overrides + node params come from gateway_config.yaml. The .so
    # path is pinned absolutely there (/ws/install/...), so we don't need to
    # resolve it via _resolve_plugin_path the way the TB3 demo does.
    _ = _resolve_plugin_path  # kept for parity / future overrides

    gateway = Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[gateway_config_file],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'headless', default_value='True',
            description='Run Gazebo without a GUI - default True for Docker/CI',
        ),
        DeclareLaunchArgument(
            'x_pose', default_value='-2.0',
            description='Robot initial X position',
        ),
        DeclareLaunchArgument(
            'y_pose', default_value='-0.5',
            description='Robot initial Y position',
        ),
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value=os.environ.get('TURTLEBOT3_MODEL', 'burger'),
        ),
        set_gz_model_path,
        gz_headless,
        gz_gui,
        spawn_robot,
        robot_state_publisher,
        nav2,
        fault_manager,
        foxglove,
        broken_lidar_node,
        broken_lidar_legacy,
        gateway,
    ])
