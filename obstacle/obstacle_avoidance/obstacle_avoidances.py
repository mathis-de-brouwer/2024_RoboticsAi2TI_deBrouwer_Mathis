import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random



class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Publisher velocity commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Sub LiDAR data
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Timer --> continuous movement
        self.timer = self.create_timer(0.1, self.move_robot)

        # Initialize state variables
        self.lidar_data = []
        self.is_turning = False
        self.turn_direction = 0  # -1 = right, 1 = left
        self.turn_angle = 0  # Random degree turn
        self.turn_timer = 0  # Turn duration

    def lidar_callback(self, msg):
        # Store LiDAR ranges/filter invalid (inf) data
        self.lidar_data = np.array([r if r < 3.5 else 3.5 for r in msg.ranges])

    def move_robot(self):
        if len(self.lidar_data) == 0:
            return

        # Twist
        cmd = Twist()
        
        #Logging in to SAO 
        self.get_logger().info(f"LiDAR data: {self.lidar_data}")

        if self.is_turning:
            
            #logging in WOW
            self.get_logger().info(f"Turning: Direction={self.turn_direction}, Timer={self.turn_timer}")
            
            # turning until turn complete
            cmd.angular.z = self.turn_direction * 0.5 # Rotate (OU-I-I-A-I-OU-I-I-I-A-I))
            self.turn_timer -= 1
            if self.turn_timer <= 0:
                self.is_turning = False  # turn complet 
            self.publisher_.publish(cmd)
            return

        # Get minimum distances
        front_distances = np.min(self.lidar_data[90:270])
        left_distances = np.min(self.lidar_data[60:120])
        right_distances = np.min(self.lidar_data[240:300])
        
        #loggin the FF14
        self.get_logger().info(f"Front: {front_distances}, Left: {left_distances}, Right: {right_distances}")

        # Decision logic (no logic :p )
        if front_distances < 0.4:  # Obstacle ahead? ahoy
            # Determine turn direction based on left/right dist
            if left_distances + 0.1 < right_distances:
                self.turn_direction = 1  # Turn left
            elif right_distances + 0.1 < left_distances:
                self.turn_direction = -1  # Turn right
            else:
                self.turn_direction = random.choice([-1, 1]) # Gambling is beautiful

            # Rando turn angle & duration
            self.turn_angle = random.randint(30, 60)  # Random angle btwn x and y
            self.turn_timer = int(self.turn_angle * 5)  #turn duration based on angle (adjustable)

            self.is_turning = True  # Start turning
            cmd.angular.z = self.turn_direction * 0.5  
        else:  # No imm obstacl
            cmd.linear.x = 0.2  # Mov forward

        # Publish velocity command
        self.publisher_.publish(cmd)


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