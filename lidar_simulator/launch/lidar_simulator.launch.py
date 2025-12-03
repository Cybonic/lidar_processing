from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'simulator_type',
            default_value='generic',
            description='Type of lidar simulator (generic, velodyne, ouster)'
        ),
        
        DeclareLaunchArgument(
            'output_topic',
            default_value='/simulated_lidar/points',
            description='Output point cloud topic'
        ),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='lidar',
            description='Frame ID for the point cloud'
        ),
        
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Publishing rate in Hz'
        ),
        
        DeclareLaunchArgument(
            'max_range',
            default_value='100.0',
            description='Maximum range in meters'
        ),
        
        DeclareLaunchArgument(
            'add_obstacles',
            default_value='true',
            description='Whether to add obstacles to the scene'
        ),

        Node(
            package='lidar_simulator',
            executable='lidar_simulator',
            name='lidar_simulator',
            parameters=[{
                'output_topic': LaunchConfiguration('output_topic'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'max_range': LaunchConfiguration('max_range'),
                'add_obstacles': LaunchConfiguration('add_obstacles'),
            }],
            output='screen'
        )
    ])