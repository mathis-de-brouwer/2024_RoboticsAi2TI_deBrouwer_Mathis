import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import time

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.get_logger().info('Obstacle Avoidance Node Started')
        
        # Constants
        self.OBSTACLE_THRESHOLD = 0.35  # Distance to start avoiding
        self.FRONT_ANGLE = 30  # ±30 degrees for front detection
        self.SIDE_ANGLE = 45  # ±45 degrees for side detection
        self.TURNING_SPEED = 0.5
        self.FORWARD_SPEED = 0.15
        
        # Create subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.safe_cmd_pub = self.create_publisher(Twist, '/safe_cmd_vel', 10)
        
        # State variables
        self.avoiding = False
        self.current_cmd = Twist()
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.turn_direction = 1

    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg  # Store the latest message
        try:
            ranges = np.array(msg.ranges)
            
            # Always check front distance
            front_indices = (
                list(range(360 - self.FRONT_ANGLE, 360)) +
                list(range(0, self.FRONT_ANGLE))
            )
            front_ranges = [
                r for i, r in enumerate(ranges)
                if i in front_indices and not np.isnan(r) and not np.isinf(r)
            ]
            
            # Update front distance
            self.front_distance = min(front_ranges) if front_ranges else float('inf')
            
            # Only check sides if we're in danger zone
            if self.front_distance <= self.OBSTACLE_THRESHOLD:
                left_indices = list(range(90 - self.SIDE_ANGLE, 90 + self.SIDE_ANGLE))
                right_indices = list(range(270 - self.SIDE_ANGLE, 270 + self.SIDE_ANGLE))
                
                left_ranges = [r for i, r in enumerate(ranges) if i in left_indices and not np.isnan(r) and not np.isinf(r)]
                right_ranges = [r for i, r in enumerate(ranges) if i in right_indices and not np.isnan(r) and not np.isinf(r)]
                
                self.left_distance = min(left_ranges) if left_ranges else float('inf')
                self.right_distance = min(right_ranges) if right_ranges else float('inf')
                
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.front_distance = float('inf')

    def cmd_vel_callback(self, msg):
        """Store the current command and check if it's safe"""
        self.current_cmd = msg
        self.check_and_avoid()

    def check_and_avoid(self):
        """Check if we need to avoid and handle accordingly"""
        if self.front_distance <= self.OBSTACLE_THRESHOLD:
            # Stop immediately if too close
            self.publish_stop()
            if not self.avoiding:
                self.avoiding = True
                self.get_logger().warn('Starting obstacle avoidance maneuver')
                self.avoid_obstacle()
        else:
            if self.avoiding:
                self.avoiding = False
                self.get_logger().info('Path is clear - You can take control now!')
            
            # Just forward the command without modification
            self.safe_cmd_pub.publish(self.current_cmd)

    def avoid_obstacle(self):
        """Simple avoidance: stop and turn until clear"""
        # First stop
        self.publish_stop()
        time.sleep(0.2)
        
        # Choose turn direction based on side distances
        self.turn_direction = 1 if self.left_distance > self.right_distance else -1
        
        # Start turning until the front distance is clear
        while self.front_distance < self.OBSTACLE_THRESHOLD:
            cmd = Twist()
            cmd.angular.z = self.TURNING_SPEED * self.turn_direction
            self.safe_cmd_pub.publish(cmd)
            time.sleep(0.2)  # Allow time for the robot to turn
            
            # Re-check the front distance
            self.lidar_callback(self.latest_lidar_msg)  # Assuming last_scan is the last received scan data
            
            # If the front distance is clear, break the loop
            if self.front_distance >= self.OBSTACLE_THRESHOLD:
                break
        
        # Stop when clear
        self.publish_stop()
        self.avoiding = False
        self.get_logger().info('Path is clear - You can take control now!')

    def publish_stop(self):
        """Publish a stop command"""
        self.safe_cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()