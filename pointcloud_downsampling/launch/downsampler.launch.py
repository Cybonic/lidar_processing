from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/ouster/points',
            description='Input point cloud topic'
        ),
        
        DeclareLaunchArgument(
            'output_topic',
            default_value='/downsampled_points',
            description='Output point cloud topic'
        ),
        
        DeclareLaunchArgument(
            'downsample_method',
            default_value='voxel',
            description='Downsampling method (voxel, uniform, random)'
        ),
        
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.1',
            description='Voxel size for voxel downsampling'
        ),
        
        DeclareLaunchArgument(
            'uniform_skip',
            default_value='10',
            description='Skip factor for uniform downsampling'
        ),
        
        DeclareLaunchArgument(
            'random_ratio',
            default_value='0.1',
            description='Ratio for random downsampling'
        ),
        
        DeclareLaunchArgument(
            'max_points',
            default_value='50000',
            description='Maximum number of points'
        ),

        Node(
            package='pointcloud_downsampling',
            executable='pointcloud_downsampler',
            name='pointcloud_downsampler',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'downsample_method': LaunchConfiguration('downsample_method'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'uniform_skip': LaunchConfiguration('uniform_skip'),
                'random_ratio': LaunchConfiguration('random_ratio'),
                'max_points': LaunchConfiguration('max_points'),
            }],
            output='screen'
        )
    ])