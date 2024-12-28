# Remove all imports except these
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
        self.OBSTACLE_THRESHOLD = 0.5
        self.SIDE_CLEARANCE = 0.3
        self.FRONT_ANGLE = 30
        self.SIDE_ANGLE = 60
        self.FORWARD_SPEED = 0.15
        self.TURNING_SPEED = 0.5
        self.MIN_TURN_ANGLE = 45
        self.MAX_TURN_ANGLE = 90
        self.BACKUP_TIME = 1.0
        
        # Create subscribers and publishers with QoS profile
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers and publishers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile)
        self.safe_cmd_pub = self.create_publisher(Twist, '/safe_cmd_vel', 10)
        
        # State variables
        self.avoiding = False
        self.current_cmd = Twist()
        self.distances = {
            'front': float('inf'),
            'left': float('inf'),
            'right': float('inf')
        }
        self.last_scan_time = self.get_clock().now()
        
        # Create safety timer
        self.timer = self.create_timer(0.1, self.safety_check)
        self.get_logger().info('Initialization complete')

    def lidar_callback(self, msg):
        try:
            self.last_scan_time = self.get_clock().now()
            ranges = np.array(msg.ranges)
            
            # Get front distances (center ±15 degrees)
            front_indices = (
                list(range(360 - self.FRONT_ANGLE, 360)) +
                list(range(0, self.FRONT_ANGLE))
            )
            front_ranges = [
                r for i, r in enumerate(ranges)
                if i in front_indices and not np.isnan(r) and not np.isinf(r)
            ]
            
            # Get left distances (60-120 degrees)
            left_ranges = [
                r for i, r in enumerate(ranges)
                if 60 <= i <= 120 and not np.isnan(r) and not np.isinf(r)
            ]
            
            # Get right distances (240-300 degrees)
            right_ranges = [
                r for i, r in enumerate(ranges)
                if 240 <= i <= 300 and not np.isnan(r) and not np.isinf(r)
            ]
            
            # Update distances
            old_front = self.distances['front']
            self.distances['front'] = min(front_ranges) if front_ranges else float('inf')
            self.distances['left'] = min(left_ranges) if left_ranges else float('inf')
            self.distances['right'] = min(right_ranges) if right_ranges else float('inf')
            
            # Log significant changes
            if abs(old_front - self.distances['front']) > 0.1:
                self.get_logger().info(
                    f'Front distance changed: {self.distances["front"]:.2f}m'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.distances = {k: float('inf') for k in self.distances}

    def cmd_vel_callback(self, msg):
        """Store the current command and check if it's safe"""
        self.get_logger().debug(f'Received command - linear: {msg.linear.x:.2f}, angular: {msg.angular.z:.2f}')
        self.current_cmd = msg
        self.check_and_avoid()

    def safety_check(self):
        """Periodic safety check"""
        if (self.get_clock().now() - self.last_scan_time).nanoseconds / 1e9 > 1.0:
            self.get_logger().warn('No recent LiDAR data - stopping robot')
            self.publish_stop()
            return
        
        self.check_and_avoid()

    def check_and_avoid(self):
        """Check for obstacles and modify command if needed"""
        if self.distances['front'] < self.OBSTACLE_THRESHOLD:
            if not self.avoiding:
                self.start_avoidance()
            return
        
        # If we were avoiding but path is now clear
        if self.avoiding and self.distances['front'] > self.OBSTACLE_THRESHOLD:
            self.avoiding = False
            self.get_logger().info('Path clear - resuming normal operation')
        
        # Forward the current command if not avoiding
        if not self.avoiding:
            self.safe_cmd_pub.publish(self.current_cmd)

    def start_avoidance(self):
        """Start the avoidance maneuver"""
        self.avoiding = True
        self.get_logger().info('Starting obstacle avoidance')
        
        # First stop
        self.publish_stop()
        
        # Check if we need to back up
        if (self.distances['front'] < self.SIDE_CLEARANCE or
            (self.distances['left'] < self.SIDE_CLEARANCE and
             self.distances['right'] < self.SIDE_CLEARANCE)):
            self.backup_and_turn()
        else:
            self.turn_to_clear_path()

    def backup_and_turn(self):
        """Back up and turn around"""
        # Back up
        cmd = Twist()
        cmd.linear.x = -self.FORWARD_SPEED
        self.safe_cmd_pub.publish(cmd)
        self.get_logger().info('Backing up')
        time.sleep(self.BACKUP_TIME)
        
        # Turn around
        self.turn_to_clear_path(force_turn=180)

    def turn_to_clear_path(self, force_turn=None):
        """Turn until a clear path is found"""
        cmd = Twist()
        
        # Decide turn direction
        if force_turn:
            turn_angle = force_turn
            direction = 1  # Always left for 180
        else:
            turn_angle = random.randint(self.MIN_TURN_ANGLE, self.MAX_TURN_ANGLE)
            if abs(self.distances['left'] - self.distances['right']) < 0.1:
                direction = random.choice([-1, 1])
            else:
                direction = 1 if self.distances['left'] > self.distances['right'] else -1
        
        # Execute turn
        cmd.angular.z = self.TURNING_SPEED * direction
        self.get_logger().info(f'Turning {"left" if direction > 0 else "right"}')
        
        # Calculate turn time based on angle
        turn_time = (turn_angle * np.pi/180) / self.TURNING_SPEED
        
        # Turn for calculated duration
        start_time = time.time()
        while time.time() - start_time < turn_time:
            self.safe_cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        # Stop turning
        self.publish_stop()

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