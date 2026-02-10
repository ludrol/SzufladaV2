import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rover_autonomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jakubgoleman12@gmail.com',
    description='Rover control package for ESP32-based rover with MQTT bridge (ROS2 Jazzy)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'distance_sensor_mqtt = rover_autonomy.distance_sensor_mqtt:main',
            'distance_sensor_serial = rover_autonomy.distance_sensor_serial:main',
            'rover_teleop = rover_autonomy.rover_teleop:main',
            'keyboard_led = rover_autonomy.keyboard_led:main',
        ],
    },
)
