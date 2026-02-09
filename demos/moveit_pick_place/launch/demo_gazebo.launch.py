"""Launch Panda robot with MoveIt 2, Gazebo Harmonic, and ros2_medkit.

Runs the Panda 7-DOF arm in Gazebo Harmonic physics simulation with
gz_ros2_control hardware interface. Provides realistic joint dynamics
and 3D visualization via Gazebo client.

This launch file starts:
  - Gazebo Harmonic with empty world (or server-only in headless mode)
  - Panda robot spawned via gz_ros2_control plugin (fixed to world)
  - ros2_control controllers (joint_state_broadcaster, arm, gripper)
  - MoveIt 2 move_group for motion planning
  - Continuous pick-and-place task loop
  - ros2_medkit gateway, fault_manager, diagnostic_bridge
  - Manipulation anomaly monitor

Prerequisites:
  ros-jazzy-ros-gz-sim, ros-jazzy-ros-gz-bridge, ros-jazzy-gz-ros2-control
"""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Package directories ──────────────────────────────────────────
    demo_pkg_dir = get_package_share_directory("moveit_medkit_demo")
    panda_config_dir = get_package_share_directory(
        "moveit_resources_panda_moveit_config"
    )

    # Config file paths — ros2_control controller definitions
    controllers_file = os.path.join(
        demo_pkg_dir, "config", "moveit_controllers.yaml"
    )
    medkit_params_file = os.path.join(
        demo_pkg_dir, "config", "medkit_params.yaml"
    )
    manifest_file = os.path.join(
        demo_pkg_dir, "config", "panda_manifest.yaml"
    )

    # Factory world file path
    factory_world = os.path.join(
        demo_pkg_dir, "worlds", "factory.sdf"
    )

    headless = LaunchConfiguration("headless", default="False")

    # ── Robot description (URDF with ros2_control + Gazebo hardware) ─
    # Use the xacro from panda_moveit_config/config/ which includes
    # ros2_control definitions and accepts ros2_control_hardware_type.
    # NOTE: The panda xacro only supports "mock_components" and "isaac"
    # hardware types. We generate with mock_components and then replace
    # the plugin name with gz_ros2_control/GazeboSimSystem.
    xacro_file = os.path.join(
        panda_config_dir, "config", "panda.urdf.xacro"
    )
    robot_description_raw = xacro.process_file(
        xacro_file,
        mappings={
            "ros2_control_hardware_type": "mock_components",
        },
    ).toxml()

    # Swap mock_components hardware plugin for Gazebo simulation plugin
    robot_description_raw = robot_description_raw.replace(
        "mock_components/GenericSystem",
        "gz_ros2_control/GazeboSimSystem",
    )

    # Inject gz_ros2_control Gazebo plugin if not already in the URDF
    if "GazeboSimROS2ControlPlugin" not in robot_description_raw:
        gz_plugin = (
            "  <gazebo>\n"
            '    <plugin filename="gz_ros2_control-system"'
            ' name="gz_ros2_control::GazeboSimROS2ControlPlugin">\n'
            f"      <parameters>{controllers_file}</parameters>\n"
            "    </plugin>\n"
            "  </gazebo>\n"
        )
        robot_description_raw = robot_description_raw.replace(
            "</robot>", gz_plugin + "</robot>"
        )

    # Anchor the robot base to the ground plane so it doesn't fall over.
    # This also provides the world→panda_link0 TF that MoveIt's
    # virtual_joint requires (published by robot_state_publisher).
    if '<link name="world"' not in robot_description_raw:
        world_joint = (
            '  <link name="world"/>\n'
            '  <joint name="world_to_panda" type="fixed">\n'
            '    <origin xyz="0 0 0.75" rpy="0 0 0"/>\n'
            '    <parent link="world"/>\n'
            '    <child link="panda_link0"/>\n'
            '  </joint>\n'
        )
        robot_description_raw = robot_description_raw.replace(
            "</robot>", world_joint + "</robot>"
        )

    # ── Inject Gazebo visual materials for robot links ───────────────
    # Gives the Panda arm a realistic industrial appearance in Gazebo
    # with white body links, dark joints, and orange accents (Franka
    # Emika brand colors).
    gazebo_materials = (
        # Base and main body links — white with subtle metallic sheen
        '  <gazebo reference="panda_link0">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.85 0.85 0.85 1</ambient>\n"
        "        <diffuse>0.92 0.92 0.92 1</diffuse>\n"
        "        <specular>0.6 0.6 0.6 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        '  <gazebo reference="panda_link1">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.85 0.85 0.85 1</ambient>\n"
        "        <diffuse>0.92 0.92 0.92 1</diffuse>\n"
        "        <specular>0.6 0.6 0.6 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        # Joints — dark anthracite grey
        '  <gazebo reference="panda_link2">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.25 0.25 0.28 1</ambient>\n"
        "        <diffuse>0.35 0.35 0.38 1</diffuse>\n"
        "        <specular>0.4 0.4 0.4 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        '  <gazebo reference="panda_link3">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.85 0.85 0.85 1</ambient>\n"
        "        <diffuse>0.92 0.92 0.92 1</diffuse>\n"
        "        <specular>0.6 0.6 0.6 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        '  <gazebo reference="panda_link4">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.25 0.25 0.28 1</ambient>\n"
        "        <diffuse>0.35 0.35 0.38 1</diffuse>\n"
        "        <specular>0.4 0.4 0.4 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        # Upper arm — white
        '  <gazebo reference="panda_link5">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.85 0.85 0.85 1</ambient>\n"
        "        <diffuse>0.92 0.92 0.92 1</diffuse>\n"
        "        <specular>0.6 0.6 0.6 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        # Wrist joint — dark
        '  <gazebo reference="panda_link6">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.25 0.25 0.28 1</ambient>\n"
        "        <diffuse>0.35 0.35 0.38 1</diffuse>\n"
        "        <specular>0.4 0.4 0.4 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        # End-effector flange — white
        '  <gazebo reference="panda_link7">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.85 0.85 0.85 1</ambient>\n"
        "        <diffuse>0.92 0.92 0.92 1</diffuse>\n"
        "        <specular>0.6 0.6 0.6 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        # Hand / gripper — dark grey with slight blue tint
        '  <gazebo reference="panda_hand">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.2 0.22 0.28 1</ambient>\n"
        "        <diffuse>0.3 0.32 0.38 1</diffuse>\n"
        "        <specular>0.45 0.45 0.5 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        # Finger tips — dark rubber-like
        '  <gazebo reference="panda_leftfinger">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.15 0.15 0.15 1</ambient>\n"
        "        <diffuse>0.22 0.22 0.22 1</diffuse>\n"
        "        <specular>0.1 0.1 0.1 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
        '  <gazebo reference="panda_rightfinger">\n'
        "    <visual>\n"
        "      <material>\n"
        "        <ambient>0.15 0.15 0.15 1</ambient>\n"
        "        <diffuse>0.22 0.22 0.22 1</diffuse>\n"
        "        <specular>0.1 0.1 0.1 1</specular>\n"
        "      </material>\n"
        "    </visual>\n"
        "  </gazebo>\n"
    )
    robot_description_raw = robot_description_raw.replace(
        "</robot>", gazebo_materials + "</robot>"
    )

    robot_description = {"robot_description": robot_description_raw}

    # ── SRDF ─────────────────────────────────────────────────────────
    srdf_file = os.path.join(panda_config_dir, "config", "panda.srdf")
    with open(srdf_file, "r") as f:
        robot_description_semantic = {
            "robot_description_semantic": f.read()
        }

    # ── Kinematics ───────────────────────────────────────────────────
    kinematics_file = os.path.join(
        panda_config_dir, "config", "kinematics.yaml"
    )
    with open(kinematics_file, "r") as f:
        kinematics_config = yaml.safe_load(f)

    # ── OMPL planning pipeline ───────────────────────────────────────
    ompl_file = os.path.join(
        panda_config_dir, "config", "ompl_planning.yaml"
    )
    with open(ompl_file, "r") as f:
        ompl_config = yaml.safe_load(f)

    # ── Joint limits ─────────────────────────────────────────────────
    joint_limits_file = os.path.join(
        panda_config_dir, "config", "joint_limits.yaml"
    )
    with open(joint_limits_file, "r") as f:
        joint_limits_config = yaml.safe_load(f)

    # ── MoveIt trajectory execution / controller manager config ──────
    moveit_ctrl_file = os.path.join(
        panda_config_dir, "config", "gripper_moveit_controllers.yaml"
    )
    with open(moveit_ctrl_file, "r") as f:
        moveit_controllers_config = yaml.safe_load(f)

    # ── Assemble move_group parameters (MoveIt-namespaced) ───────────
    move_group_params = {}
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)
    # Kinematics under robot_description_kinematics namespace
    move_group_params["robot_description_kinematics"] = kinematics_config
    # Joint limits under robot_description_planning namespace
    move_group_params["robot_description_planning"] = joint_limits_config
    # Pipeline names under planning_pipelines; OMPL config at root level
    # (MoveIt looks for "ompl.planning_plugins", not
    #  "planning_pipelines.ompl.planning_plugins")
    move_group_params["planning_pipelines"] = {
        "pipeline_names": ["ompl"],
    }
    move_group_params["ompl"] = ompl_config
    # Trajectory execution & controller manager config (top-level keys)
    move_group_params.update(moveit_controllers_config)
    move_group_params.update(
        {
            "use_sim_time": True,
            "planning_scene_monitor.publish_planning_scene": True,
            "planning_scene_monitor.publish_geometry_updates": True,
            "planning_scene_monitor.publish_state_updates": True,
        }
    )

    # ═════════════════════════════════════════════════════════════════
    return LaunchDescription(
        [
            # ── Arguments ────────────────────────────────────────────
            DeclareLaunchArgument(
                "headless",
                default_value="False",
                description="Run without GUI (True for Docker/CI)",
            ),
            # Suppress Qt when headless
            SetEnvironmentVariable(
                name="QT_QPA_PLATFORM",
                value="offscreen",
                condition=IfCondition(headless),
            ),
            # ── Add world directory to Gazebo resource path ────────
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.join(demo_pkg_dir, "worlds"),
            ),
            # ── Gazebo Harmonic (GUI) ────────────────────────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "gz_args": f"-r {factory_world}"
                }.items(),
                condition=UnlessCondition(headless),
            ),
            # ── Gazebo Harmonic (headless — server only) ─────────────
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("ros_gz_sim"),
                                "launch",
                                "gz_sim.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "gz_args": f"-r -s {factory_world}"
                }.items(),
                condition=IfCondition(headless),
            ),
            # ── Clock bridge (Gazebo → ROS 2) ────────────────────────
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
                ],
                output="screen",
            ),
            # ── Robot state publisher ────────────────────────────────
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description, {"use_sim_time": True}],
            ),
            # ── Spawn robot in Gazebo ────────────────────────────────
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-topic",
                            "robot_description",
                            "-name",
                            "panda",
                            "-allow_renaming",
                            "true",
                        ],
                        output="screen",
                    ),
                ],
            ),
            # ── Spawn controllers ────────────────────────────────────
            TimerAction(
                period=8.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "joint_state_broadcaster",
                            "-c",
                            "/controller_manager",
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda_arm_controller",
                            "-c",
                            "/controller_manager",
                        ],
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "panda_hand_controller",
                            "-c",
                            "/controller_manager",
                        ],
                    ),
                ],
            ),
            # ── MoveGroup ────────────────────────────────────────────
            TimerAction(
                period=14.0,
                actions=[
                    Node(
                        package="moveit_ros_move_group",
                        executable="move_group",
                        output="screen",
                        parameters=[move_group_params],
                    ),
                ],
            ),
            # ═════════════════════════════════════════════════════════
            # ros2_medkit stack
            # ═════════════════════════════════════════════════════════
            Node(
                package="ros2_medkit_fault_manager",
                executable="fault_manager_node",
                name="fault_manager",
                namespace="",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
            Node(
                package="ros2_medkit_diagnostic_bridge",
                executable="diagnostic_bridge_node",
                name="diagnostic_bridge",
                namespace="bridge",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "auto_generate_codes": False,
                    }
                ],
            ),
            Node(
                package="ros2_medkit_gateway",
                executable="gateway_node",
                name="ros2_medkit_gateway",
                namespace="diagnostics",
                output="screen",
                parameters=[
                    medkit_params_file,
                    {
                        "use_sim_time": True,
                        "manifest_path": manifest_file,
                    },
                ],
            ),
            # ═════════════════════════════════════════════════════════
            # Demo scripts
            # ═════════════════════════════════════════════════════════
            TimerAction(
                period=18.0,
                actions=[
                    Node(
                        package="moveit_medkit_demo",
                        executable="pick_place_loop.py",
                        name="pick_place_loop",
                        namespace="",
                        output="screen",
                        parameters=[{"use_sim_time": True}],
                    ),
                ],
            ),
            Node(
                package="moveit_medkit_demo",
                executable="manipulation_monitor.py",
                name="manipulation_monitor",
                namespace="bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),
        ]
    )
