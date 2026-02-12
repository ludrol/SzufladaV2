"""
SLAM launch file for Szuflada V2 (ROS2 Jazzy).

Launches slam_toolbox in mapping mode on top of the autonomy stack.

Usage:
    # First start the base autonomy stack:
    ros2 launch rover_autonomy autonomy_launch.py

    # Then in another terminal:
    ros2 launch rover_autonomy slam_launch.py

    # Drive the rover around with teleop to build a map.
    # Save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('rover_autonomy')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    return LaunchDescription([
        slam_toolbox,
    ])
