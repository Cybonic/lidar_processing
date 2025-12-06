from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('pcl_to_scan_pipeline')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # Static TF: odom -> os_sensor
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),

        # Delay other nodes to let clock start first
        TimerAction(
            period=3.0,
            actions=[
                # Pointcloud to Laserscan
                Node(
                    package='pointcloud_to_laserscan',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    parameters=[{
                        'sub_topic': '/ouster/points',
                        'pub_topic': '/scan',
                        'target_frame': '',
                        'min_height': 0.8,
                        'max_height': 1.2,
                        'angle_min': -3.14159,
                        'angle_max': 3.14159,
                        'range_min': 0.1,
                        'range_max': 100.0,
                        'use_sim_time': True,
                    }],
                    output='screen'
                ),

                # SLAM Toolbox
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        slam_params_file,
                        {'use_sim_time': True}
                    ],
                ),
            ]
        ),
    ])