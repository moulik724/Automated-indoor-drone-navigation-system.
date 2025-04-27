#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SimpleLidarTest(Node):
    def __init__(self):
        super().__init__('simple_lidar_test')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/car/scan',
            self.scan_callback,
            10)
        self.get_logger().info('Simple LiDAR test node started')
        
    def scan_callback(self, msg):
        # Just print the entire scan for debugging
        self.get_logger().info(f'Got scan with {len(msg.ranges)} points')
        
        # Print a few sample points
        if len(msg.ranges) > 0:
            sample_points = [0, len(msg.ranges)//4, len(msg.ranges)//2, 3*len(msg.ranges)//4]
            for i in sample_points:
                self.get_logger().info(f'Point {i}: {msg.ranges[i]}')
            
            # Find the minimum non-zero distance
            valid_ranges = [r for r in msg.ranges if r > 0.01 and r < 100.0]
            if valid_ranges:
                min_dist = min(valid_ranges)
                self.get_logger().info(f'Minimum distance: {min_dist}')
            else:
                self.get_logger().info('No valid ranges found')

def main(args=None):
    rclpy.init(args=args)
    simple_lidar_test = SimpleLidarTest()
    
    try:
        rclpy.spin(simple_lidar_test)
    except KeyboardInterrupt:
        pass
    finally:
        simple_lidar_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

