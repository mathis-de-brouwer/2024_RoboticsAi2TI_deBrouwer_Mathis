#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

class GestureController(Node):
    def __init__(self):
        super().__init__('gesture_controller')
        
        # Create subscriber for hand tracking data
        self.hand_sub = self.create_subscription(
            Float32MultiArray,
            'hand_landmarks',
            self.hand_callback,
            10)
            
        # Create publisher for Turtlebot velocity commands
        self.vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
            
        self.get_logger().info('Gesture Controller Node Started')
        
    def hand_callback(self, msg):
        # Create Twist message for robot velocity
        vel_msg = Twist()
        
        # Get hand landmarks data
        landmarks = np.array(msg.data).reshape(-1, 3)
        
        # Example gesture controls:
        # You can modify these based on your preferred gestures
        
        # Calculate hand height (using landmark 9 - middle of palm)
        hand_height = landmarks[9][1]
        
        # Calculate hand horizontal position
        hand_x = landmarks[9][0]
        
        # Forward/Backward based on hand height
        if hand_height < 0.3:  # Hand high - move forward
            vel_msg.linear.x = 0.2
        elif hand_height > 0.7:  # Hand low - move backward
            vel_msg.linear.x = -0.2
        else:
            vel_msg.linear.x = 0.0
            
        # Turning based on hand horizontal position
        if hand_x < 0.3:  # Hand left - turn left
            vel_msg.angular.z = 0.5
        elif hand_x > 0.7:  # Hand right - turn right
            vel_msg.angular.z = -0.5
        else:
            vel_msg.angular.z = 0.0
            
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