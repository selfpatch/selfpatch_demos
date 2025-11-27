"""Launch TurtleBot3 simulation with ros2_medkit gateway for discovery demo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    demo_pkg_dir = get_package_share_directory('turtlebot3_medkit_demo')

    # Path to medkit params from installed package
    medkit_params_file = os.path.join(demo_pkg_dir, 'config', 'medkit_params.yaml')

    return LaunchDescription([
        # Set TurtleBot3 model (can be overridden by environment variable)
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value=os.environ.get('TURTLEBOT3_MODEL', 'burger')
        ),

        # Launch TurtleBot3 Gazebo simulation (empty world)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'empty_world.launch.py')
            )
        ),

        # Launch ros2_medkit gateway
        Node(
            package='ros2_medkit_gateway',
            executable='gateway_node',
            name='ros2_medkit_gateway',
            output='screen',
            parameters=[medkit_params_file],
        ),
    ])
