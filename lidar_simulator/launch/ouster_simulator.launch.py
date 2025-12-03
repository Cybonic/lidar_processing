from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'add_obstacles',
            default_value='true',
            description='Whether to add obstacles to the scene'
        ),

        Node(
            package='lidar_simulator',
            executable='ouster_simulator',
            name='ouster_simulator',
            parameters=[{
                'add_obstacles': LaunchConfiguration('add_obstacles'),
            }],
            output='screen'
        )
    ])