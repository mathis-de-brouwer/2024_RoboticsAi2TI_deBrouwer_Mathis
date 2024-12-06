import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class StraightAndStop(Node):
    def __init__(self):
        super().__init__('straightandstop')

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Set QoS profile for subscriber
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.SYSTEM_DEFAULT,
            depth=10
        )

        # Subscriber for LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            self.qos_profile
        )

        # Timer to control robot movement
        self.timer = self.create_timer(0.1, self.move_robot)

        # State variable for LiDAR data
        self.lidar_data = []

    def lidar_callback(self, msg):
        """Callback to process LiDAR data."""
        # Filter invalid data and store the LiDAR ranges
        self.lidar_data = np.array([r if r < 3.5 else 3.5 for r in msg.ranges])

    def move_robot(self):
        """Robot movement logic."""
        if len(self.lidar_data) == 0:
            return  # No LiDAR data yet

        # Create a Twist message to control the robot
        cmd = Twist()

        # Get the minimum distance directly in front of the robot (90° to 270° range)
        front_distance = np.min(self.lidar_data[-30])

        # Log the front distance for debugging
        self.get_logger().info(f"Front distance: {front_distance}")

        if front_distance < 0.5:  # Stop if an obstacle is closer than 0.n meters
            cmd.linear.x = 0.0  # Stop the robot
            self.get_logger().info("Obstacle detected! Stopping.")
        else:
            cmd.linear.x = 0.2  # Move forward

        # Publish the velocity command
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StraightAndStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Shutdown gracefully when Ctrl+C is pressed
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
