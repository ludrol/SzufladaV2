"""
Launch file for the rover autonomy package (ROS2 Jazzy).

Launches the distance sensor MQTT bridge node.
The rover_teleop node is meant to be run separately in a terminal
(it requires keyboard input).

Usage:
    ros2 launch rover_autonomy rover_launch.py
    ros2 launch rover_autonomy rover_launch.py mqtt_broker:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='192.168.1.1',
        description='IP address of the MQTT broker'
    )

    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='Port of the MQTT broker'
    )

    distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='0.5',
        description='Distance threshold for LED activation (meters)'
    )

    # Distance sensor MQTT bridge node
    distance_sensor_node = Node(
        package='rover_autonomy',
        executable='distance_sensor_mqtt',
        name='distance_sensor_mqtt',
        output='screen',
        parameters=[{
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
            'distance_threshold': LaunchConfiguration('distance_threshold'),
        }]
    )

    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        distance_threshold_arg,
        distance_sensor_node,
    ])
