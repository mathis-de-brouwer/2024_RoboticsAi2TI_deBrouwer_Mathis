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
        self.SIDE_ANGLE = 45  # ±45 degrees for side detection
        self.TURNING_SPEED = 0.5
        self.TURN_STEP = 30  # Increased step for faster response
        
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
        self.last_log_time = self.get_clock().now()

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
            
            # Get side distances only when avoiding
            if self.avoiding:
                left_indices = list(range(90 - self.SIDE_ANGLE, 90 + self.SIDE_ANGLE))
                right_indices = list(range(270 - self.SIDE_ANGLE, 270 + self.SIDE_ANGLE))
                
                left_ranges = [r for i, r in enumerate(ranges) if i in left_indices and not np.isnan(r) and not np.isinf(r)]
                right_ranges = [r for i, r in enumerate(ranges) if i in right_indices and not np.isnan(r) and not np.isinf(r)]
                
                self.left_distance = min(left_ranges) if left_ranges else float('inf')
                self.right_distance = min(right_ranges) if right_ranges else float('inf')
            
            # Update front distance
            old_front = self.front_distance
            self.front_distance = min(front_ranges) if front_ranges else float('inf')
            
            # Log only significant changes and not too frequently
            now = self.get_clock().now()
            if (abs(old_front - self.front_distance) > 0.1 and 
                (now - self.last_log_time).nanoseconds / 1e9 > 1.0):  # Log at most once per second
                self.get_logger().info(f'Front distance: {self.front_distance:.2f}m')
                self.last_log_time = now
            
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
                self.get_logger().warn(f'Obstacle detected at {self.front_distance:.2f}m - stopping')
                self.avoid_obstacle()
        else:
            if self.avoiding:
                self.avoiding = False
                self.get_logger().info('Path is clear - resuming normal operation')
            self.safe_cmd_pub.publish(self.current_cmd)

    def avoid_obstacle(self):
        """Simple avoidance: stop and turn until clear"""
        # First stop
        self.publish_stop()
        time.sleep(0.2)
        
        # Choose turn direction based on side distances
        self.turn_direction = 1 if self.left_distance > self.right_distance else -1
        
        # Turn until clear
        while self.front_distance <= self.OBSTACLE_THRESHOLD:
            cmd = Twist()
            cmd.angular.z = self.TURNING_SPEED * self.turn_direction
            self.safe_cmd_pub.publish(cmd)
            time.sleep(0.2)
            
            # Stop and check
            self.publish_stop()
            time.sleep(0.1)
            
            # If we're still not clear after some time, try other direction
            if time.time() - start_time > 2.0:  # After 2 seconds
                self.turn_direction *= -1
                start_time = time.time()
        
        # Stop when clear
        self.publish_stop()
        self.avoiding = False
        self.get_logger().info('Found clear path')

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