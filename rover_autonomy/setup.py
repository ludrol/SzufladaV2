import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_autonomy'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jakubgoleman12@gmail.com',
    description='Autonomous rover control package for ESP32-based Szuflada V2 (ROS2 Jazzy)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # --- Phase 0: Manual control ---
            'distance_sensor_mqtt = rover_autonomy.distance_sensor_mqtt:main',
            'distance_sensor_serial = rover_autonomy.distance_sensor_serial:main',
            'rover_teleop = rover_autonomy.rover_teleop:main',
            'keyboard_led = rover_autonomy.keyboard_led:main',
            # --- Phase 1: Foundation ---
            'cmd_vel_bridge = rover_autonomy.cmd_vel_bridge:main',
            'odometry_node = rover_autonomy.odometry_node:main',
            'robot_description = rover_autonomy.robot_description:main',
            # --- Phase 3: Obstacle avoidance ---
            'obstacle_avoidance = rover_autonomy.obstacle_avoidance:main',
            # --- Phase 4: Wall following ---
            'wall_follower = rover_autonomy.wall_follower:main',
            # --- Phase 7: Visual tracking ---
            'visual_tracker = rover_autonomy.visual_tracker:main',
        ],
    },
)
