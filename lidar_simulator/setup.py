from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lidar_simulator'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='3D Lidar point cloud simulator for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_simulator = lidar_simulator.lidar_simulator:main',
            'velodyne_simulator = lidar_simulator.velodyne_simulator:main',
            'ouster_simulator = lidar_simulator.ouster_simulator:main',
        ],
    },
)
