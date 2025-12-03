#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class PointCloudDownsampler(Node):
    def __init__(self):
        super().__init__('pointcloud_downsampler')
        
        # Declare parameters
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/downsampled_points')
        self.declare_parameter('downsample_method', 'voxel')  # 'voxel', 'uniform', 'random'
        self.declare_parameter('voxel_size', 0.1)
        self.declare_parameter('uniform_skip', 10)
        self.declare_parameter('random_ratio', 0.1)
        self.declare_parameter('max_points', 50000)
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.downsample_method = self.get_parameter('downsample_method').get_parameter_value().string_value
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.uniform_skip = self.get_parameter('uniform_skip').get_parameter_value().integer_value
        self.random_ratio = self.get_parameter('random_ratio').get_parameter_value().double_value
        self.max_points = self.get_parameter('max_points').get_parameter_value().integer_value
        
        # QoS profile for point cloud data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            qos_profile
        )
        
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile
        )
        
        self.get_logger().info(f'PointCloud Downsampler started')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Method: {self.downsample_method}')
        
    def pointcloud_callback(self, msg):
        try:
            # Convert PointCloud2 to numpy array
            points = self.pointcloud2_to_array(msg)
            
            if len(points) == 0:
                self.get_logger().warn('Received empty point cloud')
                return
            
            # Apply downsampling
            downsampled_points = self.downsample_points(points)
            
            # Convert back to PointCloud2 and publish
            downsampled_msg = self.array_to_pointcloud2(downsampled_points, msg.header)
            self.publisher.publish(downsampled_msg)
            
            self.get_logger().debug(f'Downsampled from {len(points)} to {len(downsampled_points)} points')
            
        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')
    
    def pointcloud2_to_array(self, cloud_msg):
        """Convert PointCloud2 message to numpy array"""
        points_list = []
        
        for point in pc2.read_points(cloud_msg, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])
        
        return np.array(points_list, dtype=np.float32)
    
    def array_to_pointcloud2(self, points, header):
        """Convert numpy array to PointCloud2 message"""
        return pc2.create_cloud_xyz32(header, points.tolist())
    
    def downsample_points(self, points):
        """Apply downsampling based on the selected method"""
        
        if self.downsample_method == 'voxel':
            return self.voxel_downsample(points)
        elif self.downsample_method == 'uniform':
            return self.uniform_downsample(points)
        elif self.downsample_method == 'random':
            return self.random_downsample(points)
        else:
            self.get_logger().error(f'Unknown downsampling method: {self.downsample_method}')
            return points
    
    def voxel_downsample(self, points):
        """Voxel grid downsampling - keeps one point per voxel"""
        if len(points) == 0:
            return points
        
        # Calculate voxel indices
        voxel_indices = np.floor(points / self.voxel_size).astype(int)
        
        # Find unique voxels and their indices
        unique_voxels, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        
        # Return points corresponding to unique voxels
        downsampled = points[unique_indices]
        
        # Apply max points limit if needed
        if len(downsampled) > self.max_points:
            indices = np.random.choice(len(downsampled), self.max_points, replace=False)
            downsampled = downsampled[indices]
        
        return downsampled
    
    def uniform_downsample(self, points):
        """Uniform downsampling - keep every nth point"""
        if len(points) == 0:
            return points
        
        downsampled = points[::self.uniform_skip]
        
        # Apply max points limit if needed
        if len(downsampled) > self.max_points:
            indices = np.random.choice(len(downsampled), self.max_points, replace=False)
            downsampled = downsampled[indices]
        
        return downsampled
    
    def random_downsample(self, points):
        """Random downsampling - randomly sample a percentage of points"""
        if len(points) == 0:
            return points
        
        n_samples = int(len(points) * self.random_ratio)
        n_samples = min(n_samples, self.max_points)
        
        if n_samples >= len(points):
            return points
        
        indices = np.random.choice(len(points), n_samples, replace=False)
        return points[indices]


def main(args=None):
    rclpy.init(args=args)
    
    downsampler = PointCloudDownsampler()
    
    try:
        rclpy.spin(downsampler)
    except KeyboardInterrupt:
        pass
    finally:
        downsampler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()