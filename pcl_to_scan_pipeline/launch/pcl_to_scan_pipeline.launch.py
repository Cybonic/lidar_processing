from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Input/Output topic arguments
        DeclareLaunchArgument('input_topic', default_value='/ouster/points',
                              description='Input dense pointcloud topic'),
        DeclareLaunchArgument('downsampled_topic', default_value='/downsampled_points',
                              description='Intermediate downsampled pointcloud topic'),
        DeclareLaunchArgument('output_scan_topic', default_value='/scan',
                              description='Output laser scan topic'),
        
        # Frame arguments
        DeclareLaunchArgument('target_frame', default_value='os_sensor',
                              description='Target frame for laser scan'),
        
        # Downsampling arguments
        DeclareLaunchArgument('downsample_method', default_value='voxel',
                              description='Downsampling method: voxel, uniform, random'),
        DeclareLaunchArgument('voxel_size', default_value='0.05',
                              description='Voxel size in meters'),
        
        # Laser scan arguments
        DeclareLaunchArgument('min_height', default_value='1.0',
                              description='Minimum height for pointcloud slice'),
        DeclareLaunchArgument('max_height', default_value='1.5',
                              description='Maximum height for pointcloud slice'),
        DeclareLaunchArgument('range_min', default_value='0.1',
                              description='Minimum laser scan range'),
        DeclareLaunchArgument('range_max', default_value='100.0',
                              description='Maximum laser scan range'),

        # Node 1: Pointcloud Downsampler
        # Node(
        #     package='pointcloud_downsampling',
        #     executable='pointcloud_downsampler',
        #     name='pointcloud_downsampler',
        #     parameters=[{
        #         'input_topic': LaunchConfiguration('input_topic'),
        #         'output_topic': LaunchConfiguration('downsampled_topic'),
        #         'downsample_method': LaunchConfiguration('downsample_method'),
        #         'voxel_size': LaunchConfiguration('voxel_size'),
        #     }],
        #     output='screen'
        # ),

        # Node 2: Pointcloud to Laserscan
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                # Topic parameters (using sub_topic and pub_topic from the node)
                'sub_topic': LaunchConfiguration('input_topic'),
                'pub_topic': LaunchConfiguration('output_scan_topic'),
                # Frame parameters
                'target_frame': LaunchConfiguration('target_frame'),
                'transform_tolerance': 0.01,
                # Height filter parameters
                'min_height': LaunchConfiguration('min_height'),
                'max_height': LaunchConfiguration('max_height'),
                # Laser scan parameters
                'angle_min': -3.14159,  # -PI
                'angle_max': 3.14159,   # PI
                'angle_increment': 0.00436,  # ~0.25 degrees
                'scan_time': 0.1,
                'range_min': LaunchConfiguration('range_min'),
                'range_max': LaunchConfiguration('range_max'),
                'use_inf': True,
                'inf_epsilon': 1.0,
                'queue_size': 10,
            }],
            output='screen'
        ),
    ])
