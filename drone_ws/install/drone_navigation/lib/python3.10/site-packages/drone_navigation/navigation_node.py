import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import time

class CarNavigator(Node):
    def __init__(self):
        super().__init__('car_navigator')
        
        # Topic names - adjust these to match what's in the simulation
        self.cmd_vel_topic = '/cmd_vel'
        self.scan_topic = '/car/scan'
        
        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        
        # Navigation parameters
        self.state = "INIT"  # States: INIT, NAVIGATE
        self.start_time = time.time()
        self.init_wait_time = 3.0  # seconds to wait before starting
        
        # Add debug information
        self.get_logger().info(f'Publishing velocity commands to: {self.cmd_vel_topic}')
        self.get_logger().info(f'Subscribing to laser scans from: {self.scan_topic}')
        
        # Send test commands to verify connectivity
        self.test_command_timer = self.create_timer(1.0, self.send_test_command)
        
        # Create control timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('Car navigator initialized in INIT state. Will start moving in 3 seconds.')
    
    def send_test_command(self):
        """Send a test command to verify connectivity"""
        cmd = Twist()
        cmd.linear.x = 0.2  # Small forward velocity
        cmd.angular.z = 0.1  # Small rotational velocity
        self.cmd_pub.publish(cmd)
        self.get_logger().info('Sending test movement command')
    
    def control_loop(self):
        """Main control loop that runs based on the car's current state"""
        cmd = Twist()
        current_time = time.time()
        
        if self.state == "INIT":
            # Wait for the specified time before starting
            if current_time - self.start_time >= self.init_wait_time:
                self.state = "NAVIGATE"
                self.get_logger().info('Starting navigation sequence!')
                # Send a stronger initial movement command
                cmd.linear.x = 0.5  # Moderate forward velocity
                self.cmd_pub.publish(cmd)
            else:
                # During init, send small movements to ensure the car is responsive
                cmd.linear.x = 0.05
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f'Init state: {self.init_wait_time - (current_time - self.start_time):.1f} seconds remaining')
            
        elif self.state == "NAVIGATE":
            # Base forward movement (will be modified by scan_callback based on obstacles)
            cmd.linear.x = 0.5  # Default forward speed
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Navigate state: Moving forward')
        
    def scan_callback(self, scan_msg):
        """Process LiDAR data and navigate the car"""
        # Only navigate if we're in NAVIGATE state
        if self.state != "NAVIGATE":
            return
            
        if len(scan_msg.ranges) == 0:
            self.get_logger().warn('Empty LiDAR scan received')
            return
        
        self.get_logger().info(f'Received LIDAR scan with {len(scan_msg.ranges)} points')
            
        # Find minimum distances in front, left, right and back
        front_idx = len(scan_msg.ranges) // 4 * 0  # 0 degrees
        right_idx = len(scan_msg.ranges) // 4 * 1  # 90 degrees
        back_idx = len(scan_msg.ranges) // 4 * 2   # 180 degrees
        left_idx = len(scan_msg.ranges) // 4 * 3   # 270 degrees
        
        # Get readings in each direction (+/- 15 degrees)
        range_width = len(scan_msg.ranges) // 24  # +/- 15 degrees
        
        # Filter valid readings
        front_ranges = self.get_valid_ranges(scan_msg.ranges, front_idx, range_width)
        right_ranges = self.get_valid_ranges(scan_msg.ranges, right_idx, range_width)
        back_ranges = self.get_valid_ranges(scan_msg.ranges, back_idx, range_width)
        left_ranges = self.get_valid_ranges(scan_msg.ranges, left_idx, range_width)
        
        # Get minimum distance in each direction
        front_dist = min(front_ranges) if front_ranges else 10.0
        right_dist = min(right_ranges) if right_ranges else 10.0
        back_dist = min(back_ranges) if back_ranges else 10.0
        left_dist = min(left_ranges) if left_ranges else 10.0
        
        # Create movement command
        cmd = Twist()
        
        # Simple obstacle avoidance logic
        if front_dist < 0.8:
            # Front obstacle - stop moving forward and turn away from closest side obstacle
            cmd.linear.x = 0.0
            if left_dist < right_dist:
                cmd.angular.z = -1.0  # Turn right more strongly
                self.get_logger().info('Obstacle detected in front and left - turning right')
            else:
                cmd.angular.z = 1.0   # Turn left more strongly
                self.get_logger().info('Obstacle detected in front and right - turning left')
        else:
            # No front obstacle - go forward
            cmd.linear.x = 0.5  # Increased from 0.3 to 0.5 for more movement
            
            # Adjust to maintain distance from side walls
            if left_dist < 0.6:
                cmd.angular.z = -0.5  # Turn slightly right
                self.get_logger().info('Obstacle detected on left - turning slightly right')
            elif right_dist < 0.6:
                cmd.angular.z = 0.5   # Turn slightly left
                self.get_logger().info('Obstacle detected on right - turning slightly left')
            else:
                cmd.angular.z = 0.0   # Go straight
                self.get_logger().info('Path clear - moving forward')
        
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f'State: {self.state}, Distances - Front: {front_dist:.2f}m, Right: {right_dist:.2f}m, '
            f'Back: {back_dist:.2f}m, Left: {left_dist:.2f}m'
        )
    
    def get_valid_ranges(self, ranges, center_idx, width):
        """Get valid LiDAR readings within a range"""
        valid_ranges = []
        for i in range(center_idx - width, center_idx + width):
            # Handle wrap-around for indices
            idx = i % len(ranges)
            if ranges[idx] < 10.0 and ranges[idx] > 0.1:  # Filter out invalid readings
                valid_ranges.append(ranges[idx])
        return valid_ranges if valid_ranges else [10.0]  # Default to max range if no valid readings

def main(args=None):
    rclpy.init(args=args)
    node = CarNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

