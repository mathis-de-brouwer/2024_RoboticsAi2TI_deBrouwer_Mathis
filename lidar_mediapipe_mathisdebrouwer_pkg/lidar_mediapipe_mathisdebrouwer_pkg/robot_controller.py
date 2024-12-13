import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Constants for obstacle avoidance
        self.OBSTACLE_THRESHOLD = 0.35
        self.FRONT_ANGLE_RANGE = 15
        self.LIDAR_MAX_RANGE = 3.5
        
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
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_command)

    def lidar_callback(self, msg):
        try:
            # Get front sector distances
            ranges = np.array(msg.ranges)
            front_indices = range(360 - self.FRONT_ANGLE_RANGE, 360)
            front_indices = list(front_indices) + list(range(0, self.FRONT_ANGLE_RANGE))
            
            front_ranges = [ranges[i] for i in front_indices if not np.isnan(ranges[i]) and not np.isinf(ranges[i])]
            
            if front_ranges:
                self.front_distance = min(front_ranges)
            else:
                self.front_distance = float('inf')
                
            self.get_logger().debug(f'Front distance: {self.front_distance}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
            self.front_distance = float('inf')

    def gesture_callback(self, msg):
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

    def publish_command(self):
        cmd = Twist()
        
        # Check for obstacles
        if self.front_distance < self.OBSTACLE_THRESHOLD:
            self.get_logger().warn(f'Obstacle detected at {self.front_distance}m - Stopping!')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_publisher.publish(cmd)
            return
        
        # If no obstacles, execute the gesture command
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