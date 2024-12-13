import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Constants for obstacle avoidance
        self.OBSTACLE_THRESHOLD = 0.35
        self.SIDE_THRESHOLD = 0.30  # Minimum safe distance for sides
        self.FRONT_ANGLE_RANGE = 15
        self.SIDE_ANGLE_RANGE = 90
        self.LIDAR_MAX_RANGE = 3.5
        
        # Avoidance sequence timing
        self.TURN_TIME = 1.5  # Time to turn (seconds)
        self.FORWARD_TIME = 0.5  # Time to move forward after turning
        
        # Subscribe to hand gestures
        self.gesture_subscription = self.create_subscription(
            Int32,
            'hand_gesture',
            self.gesture_callback,
            10
        )
        
        # Subscribe to LiDAR data
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Robot control parameters
        self.FORWARD_SPEED = 0.2
        self.TURNING_SPEED = 0.5
        self.current_command = "STOP"
        self.last_command = "STOP"  # Track last command to detect changes
        self.last_fingers = -1  # Track last finger count
        
        # LiDAR data storage
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.last_obstacle_state = False  # Track last obstacle state
        
        # Avoidance state
        self.avoiding = False
        self.avoid_start_time = None
        self.turn_direction = 0  # -1 for right, 1 for left
        self.avoidance_phase = "NONE"  # NONE, TURNING, FORWARD
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_command)

    def lidar_callback(self, msg):
        try:
            ranges = np.array(msg.ranges)
            
            # Get front sector distances
            front_indices = range(360 - self.FRONT_ANGLE_RANGE, 360)
            front_indices = list(front_indices) + list(range(0, self.FRONT_ANGLE_RANGE))
            front_ranges = [ranges[i] for i in front_indices if not np.isnan(ranges[i]) and not np.isinf(ranges[i])]
            
            # Get left sector distances
            left_indices = range(45, 135)
            left_ranges = [ranges[i] for i in left_indices if not np.isnan(ranges[i]) and not np.isinf(ranges[i])]
            
            # Get right sector distances
            right_indices = range(225, 315)
            right_ranges = [ranges[i] for i in right_indices if not np.isnan(ranges[i]) and not np.isinf(ranges[i])]
            
            # Update distances
            self.front_distance = min(front_ranges) if front_ranges else float('inf')
            self.left_distance = min(left_ranges) if left_ranges else float('inf')
            self.right_distance = min(right_ranges) if right_ranges else float('inf')
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.front_distance = float('inf')
            self.left_distance = float('inf')
            self.right_distance = float('inf')

    def gesture_callback(self, msg):
        # Only process gesture if it's different from the last one
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

    def is_path_clear(self):
        """Check if the path is clear (front and sides)"""
        front_clear = self.front_distance > self.OBSTACLE_THRESHOLD
        sides_clear = (self.left_distance > self.SIDE_THRESHOLD and 
                      self.right_distance > self.SIDE_THRESHOLD)
        return front_clear and sides_clear

    def start_avoidance(self):
        # Only start avoidance if not already avoiding
        if not self.avoiding:
            self.avoiding = True
            self.avoid_start_time = time.time()
            
            # Choose turn direction based on which side has more space
            self.turn_direction = 1 if self.left_distance > self.right_distance else -1
            self.avoidance_phase = "TURNING"
            self.get_logger().info(f'Starting avoidance maneuver, turning {"left" if self.turn_direction == 1 else "right"}')
            # Stop current movement
            self.publish_stop_command()

    def continue_turning(self):
        """Reset the turning phase to continue turning"""
        self.avoid_start_time = time.time()
        self.avoidance_phase = "TURNING"
        self.get_logger().info('Continuing to turn due to obstacles')

    def publish_stop_command(self):
        cmd = Twist()
        self.cmd_vel_publisher.publish(cmd)

    def publish_command_once(self):
        cmd = Twist()
        
        if self.current_command == "FORWARD":
            cmd.linear.x = self.FORWARD_SPEED
        elif self.current_command == "LEFT":
            cmd.angular.z = self.TURNING_SPEED
        elif self.current_command == "RIGHT":
            cmd.angular.z = -self.TURNING_SPEED
        
        self.cmd_vel_publisher.publish(cmd)

    def publish_command(self):
        # Check for obstacles
        obstacle_detected = not self.is_path_clear()
        
        # If obstacle state changed
        if obstacle_detected != self.last_obstacle_state:
            self.last_obstacle_state = obstacle_detected
            if obstacle_detected:
                self.start_avoidance()
        
        # Handle avoidance sequence
        if self.avoiding:
            current_time = time.time() - self.avoid_start_time
            cmd = Twist()
            
            if self.avoidance_phase == "TURNING":
                cmd.angular.z = self.TURNING_SPEED * self.turn_direction
                
                # Only proceed to forward phase if path is clear
                if current_time >= self.TURN_TIME:
                    if self.is_path_clear():
                        self.avoidance_phase = "FORWARD"
                        self.avoid_start_time = time.time()
                        self.get_logger().info('Turn complete, moving forward')
                    else:
                        # Continue turning if path isn't clear
                        self.continue_turning()
            
            elif self.avoidance_phase == "FORWARD":
                if self.is_path_clear():
                    cmd.linear.x = self.FORWARD_SPEED
                    
                    if current_time >= self.FORWARD_TIME:
                        self.avoiding = False
                        self.avoidance_phase = "NONE"
                        self.get_logger().info('Avoidance complete, resuming normal operation')
                        # Restore last command
                        self.publish_command_once()
                        return
                else:
                    # If path becomes blocked during forward movement, restart avoidance
                    self.start_avoidance()
            
            self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 