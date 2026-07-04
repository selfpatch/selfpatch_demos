# Copyright 2026 bburda
# Apache 2.0
#
# Single launch entry point for the OTA over SOVD nav2 sensor-fix demo.
#
# Brings up, in one container:
#   - headless Gazebo (gz-sim) with the AWS small-warehouse world
#   - the Robotnik RB-Theron AMR spawned in that world, driven through
#     gz_ros2_control + a stock diff_drive_controller
#   - the full Nav2 stack (bringup_launch.py) with the warehouse map
#   - foxglove_bridge on :8765 so Foxglove Studio can render /tf, /scan, /map etc.
#   - ros2_medkit fault_manager (the gateway's /faults endpoint depends on it)
#   - the gateway with our ota_update_plugin loaded via gateway_config.yaml
#   - ros2_medkit_log_bridge + ros2_medkit_action_status_bridge, started
#     15s after boot, turning Nav2's OWN downstream failure into SOVD
#     faults (see the "Fault surfacing" comment below)
#   - health_check, exposing 4 operator-invoked Trigger operations
#     (lidar/localization/drivetrain/costmap) for differential diagnosis;
#     independent of scan_sensor_node so it survives the OTA swap
#
# /scan ownership
# ---------------
# The gz front-laser (robot/front_laser/scan) is bridged to /scan_sim, not
# /scan directly - see config/ros_gz_bridge.yaml. scan_sensor_node
# (fixed_lidar at boot, later broken_lidar once the auto-applied OTA
# regression lands) subscribes /scan_sim and republishes onto /scan: a
# clean passthrough for fixed_lidar, the real scan with a narrow blocking
# phantom wedge overlaid for broken_lidar. scan_sensor_node is the sole
# publisher on /scan that nav2 + foxglove see, whichever lidar build is
# currently applied.

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
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    demo_pkg_dir = get_package_share_directory('ota_nav2_sensor_fix_demo')

    xacro_file = os.path.join(demo_pkg_dir, 'urdf', 'warehouse_rbtheron.urdf.xacro')
    bridge_config = os.path.join(demo_pkg_dir, 'config', 'ros_gz_bridge.yaml')
    world_file = os.path.join(
        demo_pkg_dir, 'models', 'aws_small_warehouse', 'worlds', 'warehouse.sdf'
    )
    nav2_params_file = os.path.join(demo_pkg_dir, 'config', 'nav2_params.yaml')
    map_file = os.path.join(demo_pkg_dir, 'config', 'warehouse_map.yaml')

    # OTA plugin shipped via the gateway image's /etc/ros2_medkit/gateway_config.yaml.
    # The plugin itself loads when gateway_node parses that params file - we just
    # point the gateway at it via --ros-args --params-file below.
    gateway_config_file = os.environ.get(
        'OTA_DEMO_GATEWAY_CONFIG',
        '/etc/ros2_medkit/gateway_config.yaml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    headless = LaunchConfiguration('headless', default='True')

    # Spawn pose: a verified-clear north-south aisle in the warehouse map at
    # x=1.8 (checked against the committed map at the RB-Theron footprint
    # radius), facing +y (yaw pi/2) so a goal further up the aisle is a
    # straight drive. amcl.initial_pose in nav2_params.yaml MUST match this
    # pose - the map frame and the gz world frame are identity for this world.
    x_pose = LaunchConfiguration('x_pose', default='1.8')
    y_pose = LaunchConfiguration('y_pose', default='-4.2')
    yaw_pose = LaunchConfiguration('yaw_pose', default='1.5708')

    # robot_description: xacro-process our wrapper with bare joints (prefix:='').
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', "''"]),
        value_type=str,
    )

    # The world's models live alongside the warehouse models dir (also baked
    # into the image's GZ_SIM_RESOURCE_PATH by Dockerfile.gateway); append it
    # defensively for source-mounted runs.
    set_gz_model_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(demo_pkg_dir, 'models', 'aws_small_warehouse', 'models'),
    )

    # HEADLESS gz: warehouse world, server only. gz_sim.launch.py wants ONE
    # space-separated gz_args string; a list is concatenated without
    # separators so gz never sees --headless-rendering as its own flag.
    # --headless-rendering renders the gpu_lidar offscreen (EGL + Mesa) so
    # /scan publishes with no display attached.
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': '-r -s -v2 --headless-rendering ' + world_file,
            'on_exit_shutdown': 'true',
        }.items(),
        condition=IfCondition(headless),
    )

    # robot_state_publisher: publishes the URDF static + joint TF. frame_prefix=''
    # (plain empty string) keeps frames slash-free so the
    # map->odom->base_footprint->base_link->{...} TF chain stays connected for
    # Foxglove (slash-prefixed frame names render broken there).
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': '',
        }],
    )

    # Spawn the RB-Theron from /robot_description at the pinned aisle pose.
    # -z 0.15 lifts it slightly so the base mesh does not clip the warehouse
    # floor on settle.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rbtheron',
            '-x', x_pose,
            '-y', y_pose,
            '-Y', yaw_pose,
            '-z', '0.15',
        ],
    )

    # gz <-> ROS2 bridge: /clock (GZ_TO_ROS) + the front laser scan
    # robot/front_laser/scan -> /scan_sim (scan_sensor_node owns /scan from
    # there) - see config/ros_gz_bridge.yaml.
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'config_file': bridge_config,
        }],
    )

    # ros2_control spawners. The controller_manager runs INSIDE gz (the
    # gz_ros2_control plugin baked into warehouse_rbtheron.urdf.xacro), at root
    # (/controller_manager). Spawn the broadcaster first, then the
    # diff_drive_controller. Jazzy's diff_drive_controller subscribes
    # ~/cmd_vel as TwistStamped unconditionally; nav2_params.yaml already
    # publishes TwistStamped end to end (enable_stamped_cmd_vel: True on every
    # node in the chain), so no adapter/shim is needed - just remap
    # ~/cmd_vel -> /cmd_vel and ~/odom -> /odom.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='joint_state_broadcaster_spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
        ],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='diff_drive_controller_spawner',
        output='screen',
        arguments=[
            'diff_drive_controller',
            '--controller-manager', '/controller_manager',
            '--controller-ros-args',
            '-r /diff_drive_controller/cmd_vel:=/cmd_vel '
            '-r /diff_drive_controller/odom:=/odom',
        ],
    )

    # The gz_ros2_control hardware interface only exports the wheel command
    # interfaces once gz has stepped a few cycles, so the spawner above can
    # load + configure diff_drive_controller but lose the activate
    # transition - it stays inactive ("Can't accept new commands, subscriber
    # is inactive") and Nav2 then rejects every goal ("Goal rejected by
    # server"). Retry the activation until it sticks so the robot is
    # drive-ready on a cold boot with no manual
    # `ros2 control set_controller_state ... active`. Bounded to ~120s so a
    # genuinely broken hardware interface still lets the launch settle.
    controller_activator = ExecuteProcess(
        name='controller_activator',
        output='screen',
        cmd=[
            'bash',
            '-c',
            # Both spawners race the gz_ros2_control hardware readiness: whichever
            # switches first (usually joint_state_broadcaster) can time out its
            # activate and die. Without an active joint_state_broadcaster there are
            # no /joint_states, so diff_drive_controller publishes no odom TF, the
            # `odom` frame never appears and Nav2 fails every goal with "Failed to
            # make progress". Retry activating BOTH until they stick.
            "for i in $(seq 1 60); do "
            "  L=$(ros2 control list_controllers 2>/dev/null); "
            "  if echo \"$L\" | grep joint_state_broadcaster | grep -qw active "
            "     && echo \"$L\" | grep diff_drive_controller | grep -qw active; then "
            "    echo 'joint_state_broadcaster + diff_drive_controller active'; exit 0; "
            "  fi; "
            "  ros2 control set_controller_state joint_state_broadcaster active >/dev/null 2>&1; "
            "  ros2 control set_controller_state diff_drive_controller active >/dev/null 2>&1; "
            "  sleep 2; "
            "done; "
            "echo 'controller activation gave up after ~120s' >&2",
        ],
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

    fault_manager = Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'storage_type': 'memory',           # clean Faults Dashboard every run
            'healing_enabled': False,           # stored DTC: only an operator DELETE clears it
            'confirmation_threshold': -1,       # immediate confirm -> capture fires at once
            # Freeze-frame snapshots of the relevant topics at fault confirmation.
            'snapshots.enabled': True,
            # On-demand (not background cache): subscribe to each topic at
            # confirmation and wait up to timeout_sec for a message. The
            # background cache intermittently missed /cmd_vel and
            # /local_costmap/costmap (1/3 or 0/3 captured); on-demand reliably
            # grabs all three for the freeze-frame.
            'snapshots.background_capture': False,
            'snapshots.timeout_sec': 2.0,
            'snapshots.default_topics': ['/scan', '/cmd_vel', '/local_costmap/costmap'],
            # Ring-buffered MCAP around the fault (Foxglove-native, downloadable over SOVD).
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.format': 'mcap',
            'snapshots.rosbag.lazy_start': False,          # keep buffering so the pre-trigger window is captured
            'snapshots.rosbag.duration_sec': 5.0,
            'snapshots.rosbag.duration_after_sec': 2.0,
            'snapshots.rosbag.include_topics': ['/scan', '/cmd_vel', '/tf', '/tf_static', '/local_costmap/costmap'],
            'snapshots.rosbag.storage_path': '/var/lib/ros2_medkit/rosbags',
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
            # /tf publisher uses history depth 210 (15 nav2 publishers x ~14
            # transforms). foxglove_bridge defaults max_qos_depth to 25,
            # which silently drops most TF samples and breaks Foxglove's
            # TF chain reconstruction - costmaps drift off the map, robot
            # mesh floats off /scan, /amcl_pose lags. Bump to 1000 to
            # accept the full nav2 fan-in.
            {'max_qos_depth': 1000},
        ],
    )

    # broken_lidar/fixed_lidar (scan_sensor_node) own /scan by subscribing the
    # real gz laser on /scan_sim and republishing it onto /scan (fixed_lidar as
    # a clean passthrough, broken_lidar with a blocking phantom overlaid).
    # fixed_lidar boots by default; the entrypoint auto-applies broken_lidar_3_0_0
    # shortly after boot as the regressing OTA update.
    scan_sensor_node = Node(
        package='fixed_lidar',
        executable='fixed_lidar_node',
        name='scan_sensor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Operator-invoked differential-diagnosis node (4 Trigger operations:
    # lidar/localization/drivetrain/costmap health checks). Independent of
    # scan_sensor_node - it subscribes /scan directly rather than
    # depending on the broken_lidar/fixed_lidar swap, so it survives the
    # OTA update untouched and answers "broken" before the fix, "healthy"
    # after it.
    health_check_node = Node(
        package='health_check',
        executable='health_check_node',
        name='health_check',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Plugin overrides + node params come from gateway_config.yaml. The .so
    # path is pinned absolutely there (/ws/install/...), so we don't need to
    # resolve it via _resolve_plugin_path the way the earlier demo did.
    _ = _resolve_plugin_path  # kept for parity / future overrides

    gateway = Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[gateway_config_file],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    # Fault surfacing: generic ros2_medkit bridges, not a custom fault in the
    # scan nodes. The phantom (broken_lidar) blocks the path so Nav2 genuinely
    # fails on its own; these two bridges turn THAT failure into SOVD faults:
    #   - log_bridge promotes controller_server's own "Failed to make
    #     progress" /rosout ERROR to a LOG_* fault on controller-server.
    #   - action_status_bridge promotes navigate_to_pose's GoalStatus
    #     ABORTED to an ACTION_NAVIGATE_TO_POSE_ABORTED fault on bt-navigator
    #     (the action server's node).
    # Neither bridge is told about the phantom; they only see Nav2's own
    # downstream symptoms, so the operator has to investigate and correlate
    # the fault back to the recent lidar update - not read it off a
    # self-reported code.
    log_bridge = Node(
        package='ros2_medkit_log_bridge',
        executable='log_bridge_node',
        name='log_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # ERROR only, so startup INFO/WARN chatter never promotes - just
            # the controller's "Failed to make progress" once it truly stalls.
            'severity_floor': 40,
            'include_only_nodes': ['controller_server'],
            'code_prefix': 'LOG',
            'exclude_medkit_stack': True,
        }],
    )

    action_status_bridge = Node(
        package='ros2_medkit_action_status_bridge',
        executable='action_status_bridge_node',
        name='action_status_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Top-level goal only - not the BT's internal /spin, /follow_path,
            # /backup sub-actions.
            'include_only_actions': ['/navigate_to_pose'],
            'aborted_severity': 2,  # SEVERITY_ERROR
            'canceled_is_fault': False,
            'code_prefix': 'ACTION',
        }],
    )

    # Start the bridges once Nav2's lifecycle bringup + controller activation
    # have had time to settle, so they watch the real running stack rather
    # than transient startup state. Neither bridge needs this for
    # correctness (log_bridge only cares about controller_server logs after
    # a goal is sent; action_status_bridge rescans for new actions every
    # rescan_period_sec), but it keeps the fault set clean.
    bridges_after_nav2 = TimerAction(
        period=15.0,
        actions=[log_bridge, action_status_bridge],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='True',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'headless', default_value='True',
            description='Run Gazebo without a GUI - default True for Docker/CI '
            '(the only path wired here)',
        ),
        DeclareLaunchArgument(
            'x_pose', default_value='1.8',
            description='Robot initial X position (warehouse map frame)',
        ),
        DeclareLaunchArgument(
            'y_pose', default_value='-4.2',
            description='Robot initial Y position (warehouse map frame)',
        ),
        DeclareLaunchArgument(
            'yaw_pose', default_value='1.5708',
            description='Robot initial yaw, radians (warehouse map frame)',
        ),
        set_gz_model_path,
        gz_headless,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        controller_activator,
        nav2,
        fault_manager,
        foxglove,
        scan_sensor_node,
        gateway,
        bridges_after_nav2,
        health_check_node,
    ])
