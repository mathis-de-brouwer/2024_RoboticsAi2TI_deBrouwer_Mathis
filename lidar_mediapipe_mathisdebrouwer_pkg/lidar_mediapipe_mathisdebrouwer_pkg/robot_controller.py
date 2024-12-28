import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import random

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Constants for obstacle avoidance
        self.OBSTACLE_THRESHOLD = 0.5  # Increased for earlier detection
        self.SIDE_CLEARANCE_THRESHOLD = 0.3
        self.FRONT_ANGLE_RANGE = 30  # Wider front detection
        self.SIDE_ANGLE_RANGE = 60
        self.LIDAR_MAX_RANGE = 3.5
        self.MIN_TURN_ANGLE = 45  # Increased minimum turn
        self.MAX_TURN_ANGLE = 90  # Increased maximum turn
        self.TURN_TIME_MULTIPLIER = 3.0
        self.BACKUP_TIME = 1.0  # Longer backup time
        
        # Robot control parameters
        self.FORWARD_SPEED = 0.15  # Reduced from 2.0 to 0.15 for more control
        self.TURNING_SPEED = 0.5
        self.SLOW_SPEED = 0.1  # Speed when obstacles are nearby
        
        # Add QoS profile for better LiDAR data reliability
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions and publishers
        self.gesture_subscription = self.create_subscription(
            Int32, 'hand_gesture', self.gesture_callback, 10)
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to safe commands from obstacle avoidance
        self.safe_cmd_subscription = self.create_subscription(
            Twist,
            '/safe_cmd_vel',
            self.safe_cmd_callback,
            10
        )
        
        # State variables
        self.current_command = "STOP"
        self.last_command = "STOP"
        self.last_fingers = -1
        self.avoiding = False
        self.is_turning = False
        self.turn_direction = 0
        self.turn_angle = 0
        self.turn_timer = 0
        self.last_scan_time = self.get_clock().now()
        
        # LiDAR data storage
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
        # Timer for command publishing and safety checks
        self.timer = self.create_timer(0.1, self.publish_command)
        self.safety_timer = self.create_timer(1.0, self.safety_check)

    def safety_check(self):
        """Stop robot if no recent LiDAR data"""
        if (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9 > 1.0:
            self.get_logger().warn('No recent LiDAR data - stopping robot')
            self.publish_stop_command()

    def lidar_callback(self, msg):
        try:
            self.last_scan_time = self.get_clock().now()
            ranges = np.array(msg.ranges)
            
            # Get front sector distances (center ±15 degrees)
            front_start = 360 - self.FRONT_ANGLE_RANGE
            front_end = self.FRONT_ANGLE_RANGE
            front_indices = list(range(front_start, 360)) + list(range(0, front_end))
            front_ranges = [r for i, r in enumerate(ranges) if i in front_indices and not np.isnan(r) and not np.isinf(r)]
            
            # Get left sector distances (60-120 degrees)
            left_ranges = [r for i, r in enumerate(ranges) if 60 <= i <= 120 and not np.isnan(r) and not np.isinf(r)]
            
            # Get right sector distances (240-300 degrees)
            right_ranges = [r for i, r in enumerate(ranges) if 240 <= i <= 300 and not np.isnan(r) and not np.isinf(r)]
            
            # Update distances with minimum valid reading or maximum range
            old_front = self.front_distance
            self.front_distance = min(front_ranges) if front_ranges else self.LIDAR_MAX_RANGE
            
            # Log significant changes in front distance
            if abs(old_front - self.front_distance) > 0.1:  # Only log significant changes
                self.get_logger().info(f'Front distance changed from {old_front:.2f}m to {self.front_distance:.2f}m')
            
            self.left_distance = min(left_ranges) if left_ranges else self.LIDAR_MAX_RANGE
            self.right_distance = min(right_ranges) if right_ranges else self.LIDAR_MAX_RANGE
            
            # Log all distances at debug level
            self.get_logger().debug(
                f'Distances - Front: {self.front_distance:.2f}m, '
                f'Left: {self.left_distance:.2f}m, '
                f'Right: {self.right_distance:.2f}m'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.front_distance = self.LIDAR_MAX_RANGE
            self.left_distance = self.LIDAR_MAX_RANGE
            self.right_distance = self.LIDAR_MAX_RANGE

    def start_avoidance(self):
        if not self.avoiding:
            self.avoiding = True
            self.get_logger().info('Starting obstacle avoidance')
            
            # First, stop the robot
            self.publish_stop_command()
            
            # Check if we need to back up
            if (self.front_distance < self.SIDE_CLEARANCE_THRESHOLD or 
                (self.left_distance < self.SIDE_CLEARANCE_THRESHOLD and 
                 self.right_distance < self.SIDE_CLEARANCE_THRESHOLD)):
                # Back up and turn around
                cmd = Twist()
                cmd.linear.x = -self.FORWARD_SPEED
                self.cmd_vel_publisher.publish(cmd)
                self.get_logger().info('Backing up - too close to obstacles')
                time.sleep(self.BACKUP_TIME)
                self.turn_direction = 1
                self.turn_angle = 180
            else:
                # Choose turn direction based on available space
                if abs(self.left_distance - self.right_distance) < 0.1:
                    self.turn_direction = random.choice([-1, 1])
                else:
                    self.turn_direction = 1 if self.left_distance > self.right_distance else -1
                self.turn_angle = random.randint(self.MIN_TURN_ANGLE, self.MAX_TURN_ANGLE)
            
            self.turn_timer = int(self.turn_angle * self.TURN_TIME_MULTIPLIER)
            self.is_turning = True
            self.get_logger().info(
                f'Turning {self.turn_angle} degrees {"left" if self.turn_direction == 1 else "right"}'
            )

    def publish_command(self):
        # Check if we have valid LiDAR data
        if self.front_distance == float('inf'):
            self.get_logger().warn('No valid LiDAR data')
            self.publish_stop_command()
            return

        # Always check for obstacles, even during normal operation
        if self.front_distance < self.OBSTACLE_THRESHOLD:
            if not self.avoiding:
                self.get_logger().info(f'Obstacle detected at {self.front_distance:.2f}m')
                self.start_avoidance()
        
        # Handle avoidance sequence
        if self.avoiding:
            cmd = Twist()
            
            if self.is_turning:
                cmd.angular.z = self.TURNING_SPEED * self.turn_direction
                self.turn_timer -= 1
                
                if self.turn_timer <= 0:
                    self.is_turning = False
                    # Double check that path is clear
                    if self.front_distance > self.OBSTACLE_THRESHOLD:
                        self.avoiding = False
                        self.get_logger().info('Avoidance complete - path is clear')
                        self.publish_command_once()
                    else:
                        self.get_logger().info('Path still blocked - trying new direction')
                        self.start_avoidance()  # Try another direction
            else:
                # Even during avoidance, keep checking front distance
                if self.front_distance > self.OBSTACLE_THRESHOLD:
                    cmd.linear.x = self.FORWARD_SPEED * 0.5  # Move forward slowly during avoidance
                else:
                    self.start_avoidance()  # If we detect a new obstacle, start avoiding again
            
            self.cmd_vel_publisher.publish(cmd)
            return

        # If not avoiding, execute current command with continuous obstacle checking
        self.publish_command_once()

    def publish_stop_command(self):
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)

    def gesture_callback(self, msg):
        if msg.data != self.last_fingers and not self.avoiding:
            fingers = msg.data
            self.last_fingers = fingers
            
            if fingers == 5:
                self.current_command = "FORWARD"
            elif fingers == 0:
                self.current_command = "STOP"
            elif fingers == 1:
                self.current_command = "LEFT"
            elif fingers == 3:
                self.current_command = "RIGHT"
            else:
                self.current_command = "STOP"
            
            if self.current_command != self.last_command:
                self.get_logger().info(f'New gesture command: {self.current_command}')
                self.last_command = self.current_command
                self.publish_command_once()

    def publish_command_once(self):
        # Only check for very close obstacles
        if self.front_distance < 0.35:  # Only intervene when really close
            self.get_logger().info('Obstacle detected during normal operation')
            self.start_avoidance()
            return

        # Normal operation - full speed
        if not self.avoiding:
            cmd = Twist()
            
            if self.current_command == "FORWARD":
                cmd.linear.x = self.FORWARD_SPEED
            elif self.current_command == "LEFT":
                cmd.angular.z = self.TURNING_SPEED
            elif self.current_command == "RIGHT":
                cmd.angular.z = -self.TURNING_SPEED
            
            self.cmd_vel_publisher.publish(cmd)

    def safe_cmd_callback(self, msg):
        """Receive safe commands from obstacle avoidance and send to robot"""
        # Forward the safe command to the robot
        self.cmd_vel_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 