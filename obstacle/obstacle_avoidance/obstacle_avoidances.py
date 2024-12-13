import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
from typing import List
import sys



class ObstacleAvoidance(Node):
    FORWARD_SPEED = 0.2
    TURNING_SPEED = 0.5
    OBSTACLE_THRESHOLD = 0.35
    LIDAR_MAX_RANGE = 3.5
    FRONT_ANGLE_RANGE = 15
    SIDE_ANGLE_RANGE = 90
    MIN_TURN_ANGLE = 30
    MAX_TURN_ANGLE = 60
    TURN_TIME_MULTIPLIER = 5
    SIDE_CLEARANCE_THRESHOLD = 0.1  # Minimum side clearance needed
    
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # Add QoS profile for better reliability
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )

        # Add safety timer to stop robot if no LiDAR data received
        self.safety_timer = self.create_timer(1.0, self.safety_check)
        self.last_lidar_time = self.get_clock().now()
        
        # Initialize state variables
        self.lidar_data = np.array([])
        self.is_turning = False
        self.turn_direction = 0
        self.turn_angle = 0
        self.turn_timer = 0
        self.stopped_for_safety = False

    def safety_check(self):
        """Stop robot if no recent LiDAR data"""
        if (self.get_clock().now() - self.last_lidar_time).nanoseconds / 1e9 > 1.0:
            if not self.stopped_for_safety:
                self.get_logger().warn('No recent LiDAR data - stopping robot')
                self.publish_stop_command()
                self.stopped_for_safety = True
    
    def publish_stop_command(self):
        """Emergency stop command"""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

    def lidar_callback(self, msg):
        try:
            self.last_lidar_time = self.get_clock().now()
            self.stopped_for_safety = False
            
            # Filter out invalid readings and clip to range
            ranges = np.array(msg.ranges)
            valid_mask = ~np.isnan(ranges) & ~np.isinf(ranges)
            if not np.any(valid_mask):
                raise ValueError("No valid LiDAR readings")
            
            self.lidar_data = np.clip(ranges[valid_mask], 0, self.LIDAR_MAX_RANGE)
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.lidar_data = np.array([])
            self.publish_stop_command()

    def get_sector_distance(self, start_idx, end_idx):
        """Safely get minimum distance for a sector of LiDAR readings"""
        try:
            if start_idx < end_idx:
                return np.min(self.lidar_data[start_idx:end_idx])
            else:
                return np.min(np.concatenate((
                    self.lidar_data[start_idx:],
                    self.lidar_data[:end_idx]
                )))
        except Exception as e:
            self.get_logger().error(f'Error calculating sector distance: {e}')
            return self.LIDAR_MAX_RANGE

    def move_robot(self):
        if not self.lidar_data.size:
            self.get_logger().warn('No LiDAR data available')
            self.publish_stop_command()
            return

        cmd = Twist()

        try:
            if self.is_turning:
                self.get_logger().debug(
                    f'Executing turn: angle={self.turn_angle}, remaining_time={self.turn_timer}'
                )
                cmd.angular.z = self.turn_direction * self.TURNING_SPEED
                self.turn_timer -= 1
                if self.turn_timer <= 0:
                    self.is_turning = False
                self.publisher_.publish(cmd)
                return

            # Calculate distances with error handling
            front_distances = self.get_sector_distance(-self.FRONT_ANGLE_RANGE, self.FRONT_ANGLE_RANGE)
            left_distances = self.get_sector_distance(-self.SIDE_ANGLE_RANGE, -self.FRONT_ANGLE_RANGE)
            right_distances = self.get_sector_distance(self.FRONT_ANGLE_RANGE, self.SIDE_ANGLE_RANGE)

            self.get_logger().debug(
                f'Distances - Front: {front_distances:.2f}, Left: {left_distances:.2f}, '
                f'Right: {right_distances:.2f}'
            )

            if front_distances < self.OBSTACLE_THRESHOLD:
                # Check if both sides are too close
                if (left_distances < self.SIDE_CLEARANCE_THRESHOLD and 
                    right_distances < self.SIDE_CLEARANCE_THRESHOLD):
                    # Back up slightly and turn around
                    cmd.linear.x = -self.FORWARD_SPEED
                    self.turn_direction = 1  # Turn left for 180
                    self.turn_angle = 180
                else:
                    # Choose direction with more space
                    self.turn_direction = 1 if left_distances > right_distances else -1
                    self.turn_angle = random.randint(self.MIN_TURN_ANGLE, self.MAX_TURN_ANGLE)
                
                self.turn_timer = int(self.turn_angle * self.TURN_TIME_MULTIPLIER)
                self.is_turning = True
                cmd.angular.z = self.turn_direction * self.TURNING_SPEED
            else:
                # Adjust forward speed based on proximity to obstacles
                speed_factor = min(front_distances / self.OBSTACLE_THRESHOLD, 1.0)
                cmd.linear.x = self.FORWARD_SPEED * speed_factor

            self.publisher_.publish(cmd)

        except Exception as e:
            self.get_logger().error(f'Error in move_robot: {e}')
            self.publish_stop_command()


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # shutdown when Ctrl+C is pressed
        node.get_logger().info('Node stopped ok.')
    except Exception as e:
        node.get_logger().error(f'Unhandled exceptioning? -->: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()