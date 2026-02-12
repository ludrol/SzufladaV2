"""
Main autonomy launch file for Szuflada V2 (ROS2 Jazzy).

Launches the core nodes:
    - cmd_vel_bridge (Twist -> MQTT motors)
    - odometry_node (encoder MQTT -> /odom + TF)
    - robot_description (static TF frames)
    - distance_sensor_mqtt (ultrasonic MQTT -> /range)
    - depthimage_to_laserscan (depth camera -> /scan)

Usage:
    ros2 launch rover_autonomy autonomy_launch.py
    ros2 launch rover_autonomy autonomy_launch.py mqtt_broker:=192.168.1.100
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Common arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker', default_value='192.168.1.1')
    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port', default_value='1883')

    mqtt_broker = LaunchConfiguration('mqtt_broker')
    mqtt_port = LaunchConfiguration('mqtt_port')

    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package='rover_autonomy',
        executable='cmd_vel_bridge',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[{
            'mqtt_broker': mqtt_broker,
            'mqtt_port': mqtt_port,
            'max_pwm': 255,
            'max_linear_speed': 0.3,
            'wheel_base': 0.20,
            'motor_inversion': True,
            'timeout_sec': 0.5,
        }]
    )

    # Odometry
    odometry = Node(
        package='rover_autonomy',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[{
            'mqtt_broker': mqtt_broker,
            'mqtt_port': mqtt_port,
            'wheel_diameter': 0.065,
            'wheel_base': 0.20,
            'encoder_ticks_per_rev': 20,
            'motor_inversion': True,
        }]
    )

    # Robot description (static TF)
    robot_desc = Node(
        package='rover_autonomy',
        executable='robot_description',
        name='robot_description',
        output='screen',
        parameters=[{
            'ultrasonic_x': 0.10,
            'ultrasonic_y': 0.0,
            'ultrasonic_z': 0.05,
            'camera_x': 0.08,
            'camera_y': 0.0,
            'camera_z': 0.12,
        }]
    )

    # Distance sensor MQTT bridge (ultrasonic)
    distance_sensor = Node(
        package='rover_autonomy',
        executable='distance_sensor_mqtt',
        name='distance_sensor_mqtt',
        output='screen',
        parameters=[{
            'mqtt_broker': mqtt_broker,
            'mqtt_port': mqtt_port,
            'distance_threshold': 0.5,
        }]
    )

    # depthimage_to_laserscan (virtual lidar from RealSense depth)
    depth_to_scan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 10,
            'scan_time': 0.033,
            'range_min': 0.1,
            'range_max': 5.0,
            'output_frame': 'laser_frame',
        }],
        remappings=[
            ('depth', '/camera/camera/depth/image_rect_raw'),
            ('depth_camera_info', '/camera/camera/depth/camera_info'),
            ('scan', '/scan'),
        ]
    )

    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        cmd_vel_bridge,
        odometry,
        robot_desc,
        distance_sensor,
        depth_to_scan,
    ])
