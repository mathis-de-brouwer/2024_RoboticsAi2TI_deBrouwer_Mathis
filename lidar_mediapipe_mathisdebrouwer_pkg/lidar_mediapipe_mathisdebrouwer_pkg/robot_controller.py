import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Subscribe to hand gestures
        self.gesture_subscription = self.create_subscription(
            Int32,
            'hand_gesture',
            self.gesture_callback,
            10
        )
        
        # Publisher for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Robot control parameters
        self.FORWARD_SPEED = 0.2
        self.TURNING_SPEED = 0.5
        self.current_command = "STOP"
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_command)

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

    def publish_command(self):
        cmd = Twist()
        
        if self.current_command == "FORWARD":
            cmd.linear.x = self.FORWARD_SPEED
        elif self.current_command == "LEFT":
            cmd.angular.z = self.TURNING_SPEED
        elif self.current_command == "RIGHT":
            cmd.angular.z = -self.TURNING_SPEED
        # STOP command will keep all values at 0
        
        self.cmd_vel_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 