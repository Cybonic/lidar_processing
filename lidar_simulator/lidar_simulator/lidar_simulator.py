#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


class LidarSimulator(Node):
    def __init__(self):
        super().__init__('lidar_simulator')
        
        # Declare parameters
        self.declare_parameter('output_topic', '/simulated_lidar/points')
        self.declare_parameter('frame_id', 'lidar')
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('max_range', 100.0)    # meters
        self.declare_parameter('min_range', 0.5)      # meters
        self.declare_parameter('horizontal_fov', 360.0)  # degrees
        self.declare_parameter('vertical_fov', 30.0)     # degrees
        self.declare_parameter('horizontal_resolution', 0.2)  # degrees
        self.declare_parameter('vertical_resolution', 0.4)    # degrees
        self.declare_parameter('noise_std', 0.02)     # meters
        self.declare_parameter('add_ground', True)
        self.declare_parameter('add_obstacles', True)
        self.declare_parameter('ground_height', 0.0)  # meters
        self.declare_parameter('publish_tf', True)
        
        # Get parameters
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.max_range = self.get_parameter('max_range').value
        self.min_range = self.get_parameter('min_range').value
        self.horizontal_fov = self.get_parameter('horizontal_fov').value
        self.vertical_fov = self.get_parameter('vertical_fov').value
        self.horizontal_resolution = self.get_parameter('horizontal_resolution').value
        self.vertical_resolution = self.get_parameter('vertical_resolution').value
        self.noise_std = self.get_parameter('noise_std').value
        self.add_ground = self.get_parameter('add_ground').value
        self.add_obstacles = self.get_parameter('add_obstacles').value
        self.ground_height = self.get_parameter('ground_height').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # QoS profile for point cloud data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create publisher
        self.publisher = self.create_publisher(
            PointCloud2,
            self.output_topic,
            qos_profile
        )
        
        # Create TF broadcaster if needed
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create timer
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_pointcloud)
        
        # Initialize obstacles (simple geometric shapes)
        self.initialize_environment()
        
        self.get_logger().info(f'Lidar Simulator started')
        self.get_logger().info(f'Publishing to: {self.output_topic}')
        self.get_logger().info(f'Rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Range: {self.min_range}-{self.max_range} m')
        
    def initialize_environment(self):
        """Initialize the simulated environment with obstacles"""
        self.obstacles = []
        
        if self.add_obstacles:
            # Add some simple geometric obstacles
            # Box obstacles: [center_x, center_y, center_z, width, height, depth]
            self.obstacles.extend([
                {'type': 'box', 'pos': [5.0, 0.0, 1.0], 'size': [2.0, 1.0, 2.0]},
                {'type': 'box', 'pos': [-3.0, 4.0, 0.5], 'size': [1.0, 1.0, 1.0]},
                {'type': 'box', 'pos': [8.0, -3.0, 1.5], 'size': [1.5, 2.0, 3.0]},
            ])
            
            # Cylinder obstacles: [center_x, center_y, center_z, radius, height]
            self.obstacles.extend([
                {'type': 'cylinder', 'pos': [10.0, 5.0, 1.0], 'radius': 0.5, 'height': 2.0},
                {'type': 'cylinder', 'pos': [-5.0, -2.0, 0.8], 'radius': 0.3, 'height': 1.6},
            ])
            
            # Wall obstacles (long boxes)
            self.obstacles.extend([
                {'type': 'box', 'pos': [0.0, 15.0, 1.0], 'size': [20.0, 0.2, 2.0]},  # Wall
                {'type': 'box', 'pos': [15.0, 0.0, 1.0], 'size': [0.2, 20.0, 2.0]},  # Wall
            ])
    
    def publish_pointcloud(self):
        """Generate and publish a simulated point cloud"""
        try:
            points = self.generate_lidar_scan()
            
            if len(points) > 0:
                # Create PointCloud2 message
                header = self.create_header()
                pointcloud_msg = pc2.create_cloud_xyz32(header, points.tolist())
                
                # Publish
                self.publisher.publish(pointcloud_msg)
                
                # Publish TF if enabled
                if self.publish_tf:
                    self.publish_transform()
                    
                self.get_logger().debug(f'Published {len(points)} points')
            
        except Exception as e:
            self.get_logger().error(f'Error generating point cloud: {str(e)}')
    
    def generate_lidar_scan(self):
        """Generate a 3D lidar scan"""
        points = []
        
        # Calculate angle ranges
        h_angle_min = -self.horizontal_fov / 2.0
        h_angle_max = self.horizontal_fov / 2.0
        v_angle_min = -self.vertical_fov / 2.0
        v_angle_max = self.vertical_fov / 2.0
        
        # Generate rays
        h_angles = np.arange(h_angle_min, h_angle_max, self.horizontal_resolution)
        v_angles = np.arange(v_angle_min, v_angle_max, self.vertical_resolution)
        
        for h_angle in h_angles:
            for v_angle in v_angles:
                # Convert to radians
                h_rad = math.radians(h_angle)
                v_rad = math.radians(v_angle)
                
                # Calculate ray direction
                x_dir = math.cos(v_rad) * math.cos(h_rad)
                y_dir = math.cos(v_rad) * math.sin(h_rad)
                z_dir = math.sin(v_rad)
                
                # Cast ray and find intersection
                range_val = self.cast_ray(x_dir, y_dir, z_dir)
                
                if self.min_range <= range_val <= self.max_range:
                    # Calculate point position
                    x = range_val * x_dir
                    y = range_val * y_dir
                    z = range_val * z_dir
                    
                    # Add noise
                    if self.noise_std > 0:
                        noise = np.random.normal(0, self.noise_std)
                        range_val += noise
                        x = range_val * x_dir
                        y = range_val * y_dir
                        z = range_val * z_dir
                    
                    points.append([x, y, z])
        
        return np.array(points, dtype=np.float32)
    
    def cast_ray(self, x_dir, y_dir, z_dir):
        """Cast a ray and find the closest intersection"""
        min_range = self.max_range
        
        # Check ground intersection
        if self.add_ground and z_dir < 0:
            # Ray intersects ground plane at z = ground_height
            t = (self.ground_height) / (-z_dir)  # Assuming lidar at z=0
            if t > 0:
                ground_range = t
                if ground_range < min_range:
                    min_range = ground_range
        
        # Check obstacle intersections
        if self.add_obstacles:
            for obstacle in self.obstacles:
                intersection_range = self.intersect_obstacle(
                    obstacle, x_dir, y_dir, z_dir
                )
                if intersection_range > 0 and intersection_range < min_range:
                    min_range = intersection_range
        
        return min_range
    
    def intersect_obstacle(self, obstacle, x_dir, y_dir, z_dir):
        """Calculate ray-obstacle intersection"""
        if obstacle['type'] == 'box':
            return self.intersect_box(obstacle, x_dir, y_dir, z_dir)
        elif obstacle['type'] == 'cylinder':
            return self.intersect_cylinder(obstacle, x_dir, y_dir, z_dir)
        return self.max_range
    
    def intersect_box(self, box, x_dir, y_dir, z_dir):
        """Ray-box intersection using slab method"""
        pos = box['pos']
        size = box['size']
        
        # Box bounds
        min_x = pos[0] - size[0] / 2.0
        max_x = pos[0] + size[0] / 2.0
        min_y = pos[1] - size[1] / 2.0
        max_y = pos[1] + size[1] / 2.0
        min_z = pos[2] - size[2] / 2.0
        max_z = pos[2] + size[2] / 2.0
        
        # Ray origin is at (0, 0, 0)
        if abs(x_dir) < 1e-6:
            if 0 < min_x or 0 > max_x:
                return self.max_range
            t_min_x = float('-inf')
            t_max_x = float('inf')
        else:
            t1 = min_x / x_dir
            t2 = max_x / x_dir
            t_min_x = min(t1, t2)
            t_max_x = max(t1, t2)
        
        if abs(y_dir) < 1e-6:
            if 0 < min_y or 0 > max_y:
                return self.max_range
            t_min_y = float('-inf')
            t_max_y = float('inf')
        else:
            t1 = min_y / y_dir
            t2 = max_y / y_dir
            t_min_y = min(t1, t2)
            t_max_y = max(t1, t2)
        
        if abs(z_dir) < 1e-6:
            if 0 < min_z or 0 > max_z:
                return self.max_range
            t_min_z = float('-inf')
            t_max_z = float('inf')
        else:
            t1 = min_z / z_dir
            t2 = max_z / z_dir
            t_min_z = min(t1, t2)
            t_max_z = max(t1, t2)
        
        t_min = max(t_min_x, t_min_y, t_min_z)
        t_max = min(t_max_x, t_max_y, t_max_z)
        
        if t_max < 0 or t_min > t_max:
            return self.max_range
        
        t = t_min if t_min > 0 else t_max
        return t if t > 0 else self.max_range
    
    def intersect_cylinder(self, cylinder, x_dir, y_dir, z_dir):
        """Ray-cylinder intersection"""
        pos = cylinder['pos']
        radius = cylinder['radius']
        height = cylinder['height']
        
        # Translate ray to cylinder coordinate system
        cx, cy, cz = pos
        
        # Check intersection with infinite cylinder (ignoring height)
        a = x_dir * x_dir + y_dir * y_dir
        b = 2.0 * (-cx * x_dir - cy * y_dir)
        c = cx * cx + cy * cy - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return self.max_range
        
        sqrt_discriminant = math.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / (2 * a)
        t2 = (-b + sqrt_discriminant) / (2 * a)
        
        # Check which intersection is valid (considering height)
        for t in [t1, t2]:
            if t > 0:
                z_intersect = t * z_dir
                if cz - height/2 <= z_intersect <= cz + height/2:
                    return t
        
        return self.max_range
    
    def create_header(self):
        """Create message header"""
        from std_msgs.msg import Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header
    
    def publish_transform(self):
        """Publish TF transform for the lidar frame"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.frame_id
        
        # Identity transform (lidar at origin)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    simulator = LidarSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()