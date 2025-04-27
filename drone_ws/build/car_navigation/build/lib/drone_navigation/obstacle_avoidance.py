#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, Point
import numpy as np
import time
import math

class DiagonalMotionDrone(Node):
    def __init__(self):
        super().__init__('diagonal_motion_drone')
        
        # Create a subscriber for the LiDAR data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/drone/scan',  
            self.scan_callback,
            10)
            
        # Create a publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # Add IMU subscriber for better debugging
        self.imu_sub = self.create_subscription(
            Imu,
            '/drone/imu',
            self.imu_callback,
            10)
            
        # Timer for control loop - faster updates for more responsive control
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Flight parameters
        self.start_time = time.time()
        self.flight_state = "INIT"  # States: INIT, TAKEOFF, HOVER, NAVIGATE, LAND
        self.state_start_time = time.time()
        self.takeoff_duration = 15.0  # Increased to allow time to reach higher altitude
        self.hover_duration = 2.0     # Hover duration after reaching target height
        self.max_flight_time = 300.0  # 5 minutes max flight time
        
        # Define start and end points for diagonal navigation
        self.start_point = (-4.0, -4.0, 7.0)  # x, y, z (meters) - Increased height to 7m
        self.end_point = (12.0, 12.0, 7.0)      # x, y, z (meters) - Increased height to 7m
        self.landing_height = 0.05            # Final landing height
        
        # Current position estimation (will be updated based on commands sent)
        self.current_position = [0.0, 0.0, 0.15]  # x, y, z initial position
        
        # Flight control parameters
        self.max_linear_speed = 0.3     # meters/second
        self.max_angular_speed = 0.5    # radians/second
        self.hover_height = 7.0         # meters for navigation - Increased to 7m
        self.position_tolerance = 0.5   # meters, when to consider destination reached
        
        # Navigation parameters
        self.navigation_direction = self.calculate_direction_vector()
        self.navigation_progress = 0.0  # 0.0 to 1.0 progress along path
        
        # Height control parameters
        self.height_error = 0.0
        self.height_kp = 0.5           # Proportional gain for height control
        
        # Obstacle avoidance parameters
        self.safe_distance = 2.0        # Safe distance for obstacle detection
        self.ranges = []
        self.obstacle_detected = False
        self.obstacle_direction = 0.0
        self.avoiding_obstacle = False
        self.avoidance_start_time = 0.0
        self.max_avoidance_time = 5.0   # seconds to try avoidance before recalculating
        self.scan_count = 0             # Counter for scan messages received
        self.avoidance_speed = 0.4      # Speed for avoiding obstacles
        self.danger_zone = 4.0          # Distance to begin considering obstacles
        
        # Debug parameters
        self.last_scan_time = time.time()
        self.scan_interval = 0.0
        self.valid_scan_received = False
        self.scan_error_count = 0
        self.imu_received = False
        
        # Status tracking
        self.cmds_sent = 0              # Track commands sent for debugging
        self.height_reached = False     # Flag to track if target height has been reached
        
        self.get_logger().info('High altitude diagonal motion drone controller initialized')
        self.get_logger().info(f'Path: From {self.start_point} to {self.end_point} at {self.hover_height}m height')
        
    def imu_callback(self, msg):
        # Just track if we're receiving IMU data
        if not self.imu_received:
            self.imu_received = True
            self.get_logger().info("First IMU message received")
    
    def scan_callback(self, msg):
        # Calculate time since last scan for debugging
        current_time = time.time()
        self.scan_interval = current_time - self.last_scan_time
        self.last_scan_time = current_time
        
        # Track and report scan messages
        self.scan_count += 1
        if self.scan_count % 10 == 0:  # Print every 10th scan
            self.get_logger().info(f"Scan #{self.scan_count}: Interval: {self.scan_interval:.3f}s")
        
        # Check for empty scans
        if len(msg.ranges) == 0:
            self.scan_error_count += 1
            self.get_logger().error(f"Empty scan received! (Error #{self.scan_error_count})")
            return
            
        # Process LiDAR data
        self.ranges = np.array(msg.ranges)
        
        # CRITICAL DEBUG: Log raw ranges information
        if self.scan_count % 20 == 0:  # Every second
            self.get_logger().info(f"Scan ranges: min={np.min(self.ranges):.2f}, max={np.max(self.ranges):.2f}, "
                                 f"invalid count: {np.count_nonzero(~np.isfinite(self.ranges))}/{len(self.ranges)}")
        
        # Filter out invalid readings
        valid_indices = np.isfinite(self.ranges)
        if not np.any(valid_indices):
            self.scan_error_count += 1
            self.get_logger().warn(f"No valid LiDAR readings in scan! (Error #{self.scan_error_count})")
            return
        
        # Mark that we've received at least one valid scan
        self.valid_scan_received = True
        
        # Get valid ranges and find minimum distance
        valid_ranges = self.ranges[valid_indices]
        min_distance = np.min(valid_ranges)
        min_idx = np.where(self.ranges == min_distance)[0][0]
        angle_increment = msg.angle_increment
        min_angle = msg.angle_min + (min_idx * angle_increment)
        
        # Always log the minimum distance for debugging
        if self.scan_count % 10 == 0:
            self.get_logger().info(f"Min distance: {min_distance:.2f}m at angle: {math.degrees(min_angle):.1f}°")
        
        # Check if obstacle is within safe distance
        if min_distance < self.safe_distance:
            self.obstacle_detected = True
            self.obstacle_direction = min_angle
            
            # More visible warning for obstacle detection
            self.get_logger().warn(f'!!! OBSTACLE DETECTED at {min_distance:.2f}m, angle: {math.degrees(self.obstacle_direction):.1f}° !!!')
        elif min_distance < self.danger_zone:
            # Obstacle is not immediately dangerous but in the danger zone
            self.get_logger().info(f'Obstacle entering danger zone at {min_distance:.2f}m')
        else:
            self.obstacle_detected = False
    
    def calculate_direction_vector(self):
        # Calculate unit vector from start to end point
        dx = self.end_point[0] - self.start_point[0]
        dy = self.end_point[1] - self.start_point[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Return normalized direction vector
        return [dx/distance, dy/distance, 0]  # Z component is 0 as we maintain constant height
    
    def update_position_estimate(self, cmd, dt):
        # Update position estimate based on actual linear velocities
        # Note: In a real system, you'd use odometry feedback
        self.current_position[0] += cmd.linear.x * dt
        self.current_position[1] += cmd.linear.y * dt
        self.current_position[2] += cmd.linear.z * dt
        
    def calculate_progress(self):
        # Calculate progress along the path (0.0 to 1.0)
        total_distance = math.sqrt(
            (self.end_point[0] - self.start_point[0])**2 +
            (self.end_point[1] - self.start_point[1])**2
        )
        
        current_distance = math.sqrt(
            (self.current_position[0] - self.start_point[0])**2 +
            (self.current_position[1] - self.start_point[1])**2
        )
        
        # Calculate the progress but limit it to the range [0, 1]
        return min(1.0, max(0.0, current_distance / total_distance))
    
    def distance_to_goal(self):
        # Calculate Euclidean distance to the goal
        return math.sqrt(
            (self.end_point[0] - self.current_position[0])**2 +
            (self.end_point[1] - self.current_position[1])**2 +
            (self.end_point[2] - self.current_position[2])**2
        )
    
    def timer_callback(self):
        cmd = Twist()
        elapsed_time = time.time() - self.start_time
        dt = 0.05  # Time step for position estimation
        
        # Safety check - if no valid scan data received after startup grace period
        if elapsed_time > 5.0 and not self.valid_scan_received:
            self.get_logger().error("No valid scan data received! Check your LiDAR sensors.")
            # Could implement emergency behavior here
        
        # Check if flight time exceeded
        if elapsed_time > self.max_flight_time:
            self.get_logger().info('Maximum flight time exceeded. Landing the drone.')
            self.flight_state = "LAND"
            self.state_start_time = time.time()
        
        # CRITICAL: Handle obstacle detection with priority and additional logging
        if self.obstacle_detected and self.flight_state not in ["INIT", "TAKEOFF", "HOVER", "LAND"]:
            self.get_logger().warn(f"EXECUTING OBSTACLE AVOIDANCE - State: {self.flight_state}")
            self.handle_obstacle_avoidance(cmd)
        else:
            # Execute the current flight state
            self.execute_current_state(cmd)
        
        # Update position estimate based on commands
        self.update_position_estimate(cmd, dt)
        
        # Calculate navigation progress
        self.navigation_progress = self.calculate_progress()
        
        # Increment command counter and publish
        self.cmds_sent += 1
        if self.cmds_sent % 20 == 0:  # Log every ~1 second
            self.get_logger().info(
                f'State: {self.flight_state}, Commands: {self.cmds_sent}, ' +
                f'Position: ({self.current_position[0]:.2f}, {self.current_position[1]:.2f}, {self.current_position[2]:.2f}), ' +
                f'Progress: {self.navigation_progress:.2f}, ' +
                f'Obstacle detection: {"ACTIVE" if self.obstacle_detected else "inactive"}'
            )
            
        self.cmd_vel_pub.publish(cmd)
    
    def execute_current_state(self, cmd):
        # If we're starting a new state
        if self.flight_state == "INIT":
            # Initialize with takeoff
            self.flight_state = "TAKEOFF"
            self.state_start_time = time.time()
            self.get_logger().info(f"Starting takeoff to height of {self.hover_height}m")
        
        # Get elapsed time in current state
        state_elapsed = time.time() - self.state_start_time
        
        # Execute the current state
        if self.flight_state == "TAKEOFF":
            self.execute_takeoff(cmd, state_elapsed)
            
            # Check if we've reached the target height
            if self.current_position[2] >= self.hover_height - 0.1:
                if not self.height_reached:
                    self.height_reached = True
                    self.get_logger().info(f"Target height of {self.hover_height}m reached!")
                
                # Transition to hover state after reaching target height
                if state_elapsed >= self.takeoff_duration:
                    self.flight_state = "HOVER"
                    self.state_start_time = time.time()
                    self.get_logger().info(f"Takeoff complete. Hovering at {self.current_position[2]:.2f}m.")
        
        elif self.flight_state == "HOVER":
            # Maintain position and height during hover
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            
            # Height control during hover
            self.height_error = self.hover_height - self.current_position[2]
            cmd.linear.z = self.height_error * self.height_kp
            
            # After hover duration, start navigation
            if state_elapsed >= self.hover_duration:
                self.flight_state = "NAVIGATE"
                self.state_start_time = time.time()
                self.get_logger().info("Starting diagonal navigation.")
        
        elif self.flight_state == "NAVIGATE":
            # Check if we've reached the destination
            if self.distance_to_goal() < self.position_tolerance:
                self.flight_state = "LAND"
                self.state_start_time = time.time()
                self.get_logger().info("Destination reached. Preparing to land.")
            else:
                # Calculate direction to goal
                dx = self.end_point[0] - self.current_position[0]
                dy = self.end_point[1] - self.current_position[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Normalize direction vector
                if distance > 0:
                    dx /= distance
                    dy /= distance
                
                # Set velocities in the direction of the goal
                cmd.linear.x = dx * self.max_linear_speed
                cmd.linear.y = dy * self.max_linear_speed
                
                # Height control - maintain the hover height at 7m
                self.height_error = self.hover_height - self.current_position[2]
                cmd.linear.z = self.height_error * self.height_kp
                
                self.get_logger().debug(f"Navigating to goal, dx={dx:.2f}, dy={dy:.2f}, h_err={self.height_error:.2f}, dist={distance:.2f}")
        
        elif self.flight_state == "LAND":
            # Prepare target landing position (x,y coordinates of end point with landing height)
            landing_x = self.end_point[0]
            landing_y = self.end_point[1]
            
            # Move to landing position horizontally while descending
            dx = landing_x - self.current_position[0]
            dy = landing_y - self.current_position[1]
            
            # Slow horizontal approach during landing
            cmd.linear.x = dx * 0.1  # Reduced speed for gentle landing
            cmd.linear.y = dy * 0.1  # Reduced speed for gentle landing
            
            # Controlled descent with stages based on current height
            if self.current_position[2] > 5.0:
                cmd.linear.z = -0.4  # Faster initial descent from high altitude
            elif self.current_position[2] > 2.0:
                cmd.linear.z = -0.3  # Medium-fast descent from high altitude
            elif self.current_position[2] > 1.0:
                cmd.linear.z = -0.2  # Medium descent rate
            elif self.current_position[2] > 0.2:
                cmd.linear.z = -0.1  # Slow descent
            else:
                cmd.linear.z = -0.05  # Very slow final approach
                
            self.get_logger().debug(f"Landing at height: {self.current_position[2]:.2f}, target: {self.landing_height}")
            
            # If we've reached landing height, stop everything
            if self.current_position[2] <= self.landing_height + 0.05:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.linear.z = 0.0
                cmd.angular.z = 0.0
                self.get_logger().info("Landing complete")
                self.timer.cancel()  # Stop the timer
    
    def execute_takeoff(self, cmd, state_elapsed):
        # Takeoff procedure with stronger vertical thrust for higher altitude
        # Calculate height error
        height_error = self.hover_height - self.current_position[2]
        
        # Apply stronger vertical thrust at beginning for initial lift to reach 7m
        if state_elapsed < 3.0:
            cmd.linear.z = 0.8  # Stronger initial thrust for higher altitude
        else:
            # Then use proportional control for height with a minimum thrust
            cmd.linear.z = height_error * self.height_kp
            
            # Add minimum upward thrust to ensure we keep climbing if not at target height
            if cmd.linear.z < 0.3 and height_error > 0.1:
                cmd.linear.z = 0.3
        
        # Stabilize in place
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        
        # Log takeoff progress
        if int(state_elapsed) != int(state_elapsed - 0.05) and state_elapsed % 1.0 < 0.06:
            self.get_logger().info(f'Takeoff progress: {state_elapsed:.1f}s, height: {self.current_position[2]:.2f}m, target: {self.hover_height}m')
    
    def handle_obstacle_avoidance(self, cmd):
        # If we're beginning avoidance, set the start time
        if not self.avoiding_obstacle:
            self.avoiding_obstacle = True
            self.avoidance_start_time = time.time()
            self.get_logger().info("Starting obstacle avoidance maneuver")
        
        # Check if we've been avoiding for too long
        avoidance_elapsed = time.time() - self.avoidance_start_time
        if avoidance_elapsed > self.max_avoidance_time:
            # Try a different approach - turn more aggressively and increase avoidance speed
            self.get_logger().info(f"Avoidance taking too long ({avoidance_elapsed:.1f}s). Adjusting strategy.")
            self.avoidance_speed += 0.1  # Increase avoidance speed
            self.avoidance_start_time = time.time()  # Reset timer
        
        # Determine which direction to move (opposite to obstacle)
        is_left = self.obstacle_direction > 0
        
        # First rotate away from obstacle
        cmd.angular.z = self.max_angular_speed * (-1.0 if is_left else 1.0)
        
        # Then actually move away - THIS IS THE KEY ADDITION
        # Compute perpendicular direction to obstacle for evasion
        perpendicular_angle = self.obstacle_direction + (math.pi/2 if is_left else -math.pi/2)
        
        # Set movement in the perpendicular direction to avoid the obstacle
        cmd.linear.x = self.avoidance_speed * math.cos(perpendicular_angle)  # Move sideways away from obstacle
        cmd.linear.y = self.avoidance_speed * math.sin(perpendicular_angle)  # Move sideways away from obstacle
        
        # Height control during obstacle avoidance - maintain the 7m height
        # Option to significantly increase height during avoidance for additional safety
        target_height = self.hover_height + 1.0  # Add 1.0m to flying height during avoidance for more clearance
        self.height_error = target_height - self.current_position[2]
        cmd.linear.z = self.height_error * self.height_kp * 2.0  # Double the height control gain for faster response
        
        self.get_logger().warn(f'AVOIDING OBSTACLE at angle {math.degrees(self.obstacle_direction):.1f}°, ' + 
                              f'moving in direction {math.degrees(perpendicular_angle):.1f}°, ' +
                              f'speed: {self.avoidance_speed:.2f}m/s, target height: {target_height:.1f}m')
        
        # If obstacle is no longer detected, resume navigation
        if not self.obstacle_detected:
            self.avoiding_obstacle = False
            self.avoidance_speed = 0.4  # Reset avoidance speed
            self.get_logger().info("Obstacle cleared, resuming navigation")
            
def main(args=None):
    rclpy.init(args=args)
    drone_controller = DiagonalMotionDrone()
    
    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure a safe landing on shutdown
        final_cmd = Twist()
        final_cmd.linear.z = -0.2
        drone_controller.cmd_vel_pub.publish(final_cmd)
        time.sleep(1.0)
        drone_controller.cmd_vel_pub.publish(Twist())
        
        drone_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

