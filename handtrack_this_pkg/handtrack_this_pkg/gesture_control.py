#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        
        # Create subscriber for hand gesture data
        self.hand_sub = self.create_subscription(
            Int32,
            'hand_gesture',
            self.gesture_callback,
            10)
            
        # Create publisher for Turtlebot velocity commands
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
            
        self.get_logger().info('Gesture Controller Node Started')
    
    def gesture_callback(self, msg):
        vel_msg = Twist()
        
        # Get number of fingers up
        fingers_up = msg.data
        
        # Open hand (5 fingers) - Move forward
        if fingers_up == 5:
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 0.0
        
        # Closed hand (0 fingers) - Stop
        elif fingers_up == 0:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
        
        # 1 finger - Turn left
        elif fingers_up == 1:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.5  # Positive value for left turn
        
        # 3 fingers - Turn right
        elif fingers_up == 3:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = -0.5  # Negative value for right turn
        
        # Publish velocity command
        self.vel_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    gesture_controller = GestureController()
    rclpy.spin(gesture_controller)
    gesture_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 