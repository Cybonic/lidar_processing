#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from .lidar_simulator import LidarSimulator


class OusterSimulator(LidarSimulator):
    """Simulator specifically configured for Ouster OS1-64 characteristics"""
    
    def __init__(self):
        super().__init__()
        
        # Override parameters with Ouster OS1-64 specifications
        self.set_parameters([
            rclpy.parameter.Parameter('output_topic', rclpy.Parameter.Type.STRING, '/ouster/points'),
            rclpy.parameter.Parameter('frame_id', rclpy.Parameter.Type.STRING, 'ouster'),
            rclpy.parameter.Parameter('publish_rate', rclpy.Parameter.Type.DOUBLE, 10.0),
            rclpy.parameter.Parameter('max_range', rclpy.Parameter.Type.DOUBLE, 120.0),
            rclpy.parameter.Parameter('min_range', rclpy.Parameter.Type.DOUBLE, 0.3),
            rclpy.parameter.Parameter('horizontal_fov', rclpy.Parameter.Type.DOUBLE, 360.0),
            rclpy.parameter.Parameter('vertical_fov', rclpy.Parameter.Type.DOUBLE, 33.2),  # +16.6° to -16.6°
            rclpy.parameter.Parameter('horizontal_resolution', rclpy.Parameter.Type.DOUBLE, 0.35),  # ~1024 points per revolution
            rclpy.parameter.Parameter('vertical_resolution', rclpy.Parameter.Type.DOUBLE, 0.52),   # 64 channels
            rclpy.parameter.Parameter('noise_std', rclpy.Parameter.Type.DOUBLE, 0.02),
        ])
        
        self.get_logger().info('Ouster OS1-64 Simulator configured')


def main(args=None):
    rclpy.init(args=args)
    
    simulator = OusterSimulator()
    
    try:
        rclpy.spin(simulator)
    except KeyboardInterrupt:
        pass
    finally:
        simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()