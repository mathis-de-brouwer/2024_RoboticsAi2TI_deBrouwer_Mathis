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
        self.OBSTACLE_THRESHOLD = 0.3  # Distance to start avoiding
        self.FRONT_ANGLE = 30  # ±30 degrees for front detection
        self.TURNING_SPEED = 0.5
        self.TURN_TIME = 1.0  # Time to turn (seconds)
        
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

    def lidar_callback(self, msg):
        try:
            ranges = np.array(msg.ranges)
            
            # Get front distances (center ±30 degrees)
            front_indices = (
                list(range(360 - self.FRONT_ANGLE, 360)) +
                list(range(0, self.FRONT_ANGLE))
            )
            front_ranges = [
                r for i, r in enumerate(ranges)
                if i in front_indices and not np.isnan(r) and not np.isinf(r)
            ]
            
            # Update front distance
            old_front = self.front_distance
            self.front_distance = min(front_ranges) if front_ranges else float('inf')
            
            # Log significant changes
            if abs(old_front - self.front_distance) > 0.1:
                self.get_logger().info(f'Front distance: {self.front_distance:.2f}m')
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.front_distance = float('inf')

    def cmd_vel_callback(self, msg):
        """Store the current command and check if it's safe"""
        self.current_cmd = msg
        self.check_and_avoid()

    def check_and_avoid(self):
        """Simple obstacle avoidance: if obstacle detected, stop and turn"""
        if self.front_distance < self.OBSTACLE_THRESHOLD:
            if not self.avoiding:
                self.avoiding = True
                self.get_logger().info('Obstacle detected - starting avoidance')
                self.avoid_obstacle()
        else:
            if self.avoiding:
                self.avoiding = False
                self.get_logger().info('Path clear - resuming normal operation')
            self.safe_cmd_pub.publish(self.current_cmd)

    def avoid_obstacle(self):
        """Stop, turn, and check if path is clear"""
        # First stop
        self.publish_stop()
        time.sleep(0.5)  # Short pause
        
        # Keep turning until we find a clear path
        while self.front_distance < self.OBSTACLE_THRESHOLD:
            # Turn
            cmd = Twist()
            cmd.angular.z = self.TURNING_SPEED
            self.safe_cmd_pub.publish(cmd)
            self.get_logger().info('Turning to find clear path...')
            time.sleep(self.TURN_TIME)
            
            # Stop and check
            self.publish_stop()
            time.sleep(0.5)  # Wait for stable readings
        
        # Path is clear, resume normal operation
        self.avoiding = False
        self.get_logger().info('Found clear path - resuming normal operation')

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