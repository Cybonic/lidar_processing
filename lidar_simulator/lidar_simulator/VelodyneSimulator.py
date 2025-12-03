#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from .lidar_simulator import LidarSimulator


class VelodyneSimulator(LidarSimulator):
    """Simulator specifically configured for Velodyne VLP-16 characteristics"""
    
    def __init__(self):
        super().__init__()
        
        # Override parameters with Velodyne VLP-16 specifications
        self.set_parameters([
            rclpy.parameter.Parameter('output_topic', rclpy.Parameter.Type.STRING, '/velodyne_points'),
            rclpy.parameter.Parameter('frame_id', rclpy.Parameter.Type.STRING, 'velodyne'),
            rclpy.parameter.Parameter('publish_rate', rclpy.Parameter.Type.DOUBLE, 10.0),
            rclpy.parameter.Parameter('max_range', rclpy.Parameter.Type.DOUBLE, 100.0),
            rclpy.parameter.Parameter('min_range', rclpy.Parameter.Type.DOUBLE, 0.4),
            rclpy.parameter.Parameter('horizontal_fov', rclpy.Parameter.Type.DOUBLE, 360.0),
            rclpy.parameter.Parameter('vertical_fov', rclpy.Parameter.Type.DOUBLE, 30.0),  # +15° to -15°
            rclpy.parameter.Parameter('horizontal_resolution', rclpy.Parameter.Type.DOUBLE, 0.2),  # ~1800 points per revolution
            rclpy.parameter.Parameter('vertical_resolution', rclpy.Parameter.Type.DOUBLE, 2.0),   # 16 channels
            rclpy.parameter.Parameter('noise_std', rclpy.Parameter.Type.DOUBLE, 0.03),
        ])
        
        self.get_logger().info('Velodyne VLP-16 Simulator configured')


def main(args=None):
    rclpy.init(args=args)
    
    simulator = VelodyneSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()