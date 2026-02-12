"""
Nav2 launch file for Szuflada V2 (ROS2 Jazzy).

Launches the full Nav2 navigation stack with a pre-built map.

Usage:
    # First start the base autonomy stack:
    ros2 launch rover_autonomy autonomy_launch.py

    # Then launch Nav2 with the map:
    ros2 launch rover_autonomy nav2_launch.py map:=/root/map.yaml

    # Use RViz2 to set navigation goals.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonomy')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(os.path.expanduser('~'), 'map.yaml'),
        description='Full path to the map YAML file'
    )

    # Use nav2_bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
            'use_sim_time': 'false',
        }.items()
    )

    return LaunchDescription([
        map_arg,
        nav2_launch,
    ])
