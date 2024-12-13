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
        
        # LiDAR data storage
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        
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
            
            # Get left sector distances (wider angle for better decision making)
            left_indices = range(45, 135)  # 90° ± 45°
            left_ranges = [ranges[i] for i in left_indices if not np.isnan(ranges[i]) and not np.isinf(ranges[i])]
            
            # Get right sector distances (wider angle for better decision making)
            right_indices = range(225, 315)  # 270° ± 45°
            right_ranges = [ranges[i] for i in right_indices if not np.isnan(ranges[i]) and not np.isinf(ranges[i])]
            
            # Update distances
            self.front_distance = min(front_ranges) if front_ranges else float('inf')
            self.left_distance = min(left_ranges) if left_ranges else float('inf')
            self.right_distance = min(right_ranges) if right_ranges else float('inf')
            
            self.get_logger().debug(f'Distances - Front: {self.front_distance:.2f}, Left: {self.left_distance:.2f}, Right: {self.right_distance:.2f}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.front_distance = float('inf')
            self.left_distance = float('inf')
            self.right_distance = float('inf')

    def gesture_callback(self, msg):
        # Only accept new gestures if not avoiding obstacles
        if not self.avoiding:
            fingers = msg.data
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
            
            self.get_logger().info(f'Received gesture command: {self.current_command}')

    def start_avoidance(self):
        self.avoiding = True
        self.avoid_start_time = time.time()
        # Choose turn direction based on available space
        self.turn_direction = 1 if self.left_distance > self.right_distance else -1
        self.avoidance_phase = "TURNING"
        self.get_logger().info(f'Starting avoidance maneuver, turning {"left" if self.turn_direction == 1 else "right"}')

    def publish_command(self):
        cmd = Twist()
        
        # Check for obstacles and handle avoidance
        if self.front_distance < self.OBSTACLE_THRESHOLD:
            if not self.avoiding:
                self.start_avoidance()
        
        # Handle avoidance sequence
        if self.avoiding:
            current_time = time.time() - self.avoid_start_time
            
            if self.avoidance_phase == "TURNING":
                # Execute turn
                cmd.angular.z = self.TURNING_SPEED * self.turn_direction
                
                # Check if turn is complete
                if current_time >= self.TURN_TIME:
                    self.avoidance_phase = "FORWARD"
                    self.avoid_start_time = time.time()  # Reset timer for forward phase
                    self.get_logger().info('Turn complete, moving forward')
            
            elif self.avoidance_phase == "FORWARD":
                # Move forward if no obstacle in front
                if self.front_distance > self.OBSTACLE_THRESHOLD:
                    cmd.linear.x = self.FORWARD_SPEED
                    
                    # Check if forward movement is complete
                    if current_time >= self.FORWARD_TIME:
                        self.avoiding = False
                        self.avoidance_phase = "NONE"
                        self.get_logger().info('Avoidance complete, resuming normal operation')
                else:
                    # If we encounter another obstacle during forward movement, restart avoidance
                    self.start_avoidance()
            
            self.cmd_vel_publisher.publish(cmd)
            return
        
        # If not avoiding obstacles, execute gesture commands
        if self.current_command == "FORWARD":
            cmd.linear.x = self.FORWARD_SPEED
            self.get_logger().info('Moving forward')
        elif self.current_command == "LEFT":
            cmd.angular.z = self.TURNING_SPEED
            self.get_logger().info('Turning left')
        elif self.current_command == "RIGHT":
            cmd.angular.z = -self.TURNING_SPEED
            self.get_logger().info('Turning right')
        else:
            self.get_logger().info('Stopped')
        
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 