import cv2
import mediapipe as mp
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
from typing import List
import sys

class handDetector(Node):
    def __init__(self, mode=False, maxHands=2, detectionCon=0.5, trackCon=0.5):
        super().__init__('hand_detector')
        self.mode = mode
        self.maxHands = maxHands
        self.detectionCon = detectionCon
        self.trackCon = trackCon

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, 
                                      min_detection_confidence=0.7,
                                      min_tracking_confidence=0.7)
        self.mpDraw = mp.solutions.drawing_utils
        
        # Create publisher for gesture commands
        self.gesture_publisher = self.create_publisher(Int32, 'hand_gesture', 10)

    def findHands(self,img, draw=True):
        imgRGB = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        if self.results.multi_hand_landmarks:
            for handLms in self.results.multi_hand_landmarks:
                if draw:
                    self.mpDraw.draw_landmarks(img,handLms,self.mpHands.HAND_CONNECTIONS)
        return img

    def findPosition(self, img, handno=0, draw=True):
        lmList = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handno]
            for id,lm in enumerate(myHand.landmark):
                h,w,c = img.shape
                cx,cy = int(lm.x*w), int(lm.y*h)
                lmList.append([id,cx,cy])
                if draw:
                    cv2.circle(img,(cx,cy),5,(255,0,255),cv2.FILLED)
        return lmList

    def countFingers(self, lmList):
        if len(lmList) == 0:
            return 0
        
        fingers = []
        
        # Finger tips IDs
        tipIds = [4, 8, 12, 16, 20]  # thumb, index, middle, ring, pinky
        
        # For thumb
        if lmList[4][1] < lmList[3][1]:
            fingers.append(1)
        else:
            fingers.append(0)
            
        # For other 4 fingers
        for id in range(1, 5):
            if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        
        return sum(fingers)

    def get_command_text(self, fingers_up):
        if fingers_up == 5:
            return "FORWARD"
        elif fingers_up == 0:
            return "STOP"
        elif fingers_up == 1:
            return "TURN LEFT"
        elif fingers_up == 3:
            return "TURN RIGHT"
        return "NO COMMAND"

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

        # Add subscription to hand gestures
        self.gesture_subscription = self.create_subscription(
            Int32,
            'hand_gesture',
            self.gesture_callback,
            10
        )
        
        # Add gesture control state
        self.current_gesture = 0
        self.manual_control = False

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

    def gesture_callback(self, msg):
        self.current_gesture = msg.data
        self.manual_control = True
        # Reset manual control after 2 seconds of no gestures
        self.create_timer(2.0, self.reset_manual_control)

    def reset_manual_control(self):
        self.manual_control = False

    def move_robot(self):
        if not self.lidar_data.size:
            self.get_logger().warn('No LiDAR data available')
            self.publish_stop_command()
            return

        cmd = Twist()

        try:
            # Get minimum distances
            front_distances = self.get_sector_distance(-self.FRONT_ANGLE_RANGE, self.FRONT_ANGLE_RANGE)
            
            # Emergency stop if too close to obstacle
            if front_distances < self.OBSTACLE_THRESHOLD:
                self.publish_stop_command()
                return
                
            # Handle manual control with gesture
            if self.manual_control:
                if self.current_gesture == 5:  # Forward
                    cmd.linear.x = self.FORWARD_SPEED
                elif self.current_gesture == 1:  # Left
                    cmd.angular.z = self.TURNING_SPEED
                elif self.current_gesture == 3:  # Right
                    cmd.angular.z = -self.TURNING_SPEED
                # gesture 0 means stop (cmd stays at 0)
                
                self.publisher_.publish(cmd)
                return

            # If no manual control, continue with autonomous obstacle avoidance
            # Calculate distances with error handling
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
    
    # Create the hand detector node
    detector = handDetector()
    
    # Initialize camera
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        detector.get_logger().error('Could not open camera')
        return
        
    pTime = 0
    
    try:
        while rclpy.ok():
            # Process any pending callbacks
            rclpy.spin_once(detector, timeout_sec=0)
            
            # Read camera frame
            success, img = cap.read()
            if not success:
                detector.get_logger().warn('Failed to read camera frame')
                continue
                
            # Flip the image horizontally for a later selfie-view display
            img = cv2.flip(img, 1)
            
            # Process hand detection
            img = detector.findHands(img)
            lmList = detector.findPosition(img)
            
            # Process gestures and publish commands
            command_text = "NO HAND DETECTED"
            if len(lmList) != 0:
                fingers_up = detector.countFingers(lmList)
                command_text = detector.get_command_text(fingers_up)
                
                # Publish gesture command
                msg = Int32()
                msg.data = fingers_up
                detector.gesture_publisher.publish(msg)
                
                # Draw circles on fingertips for visualization
                tipIds = [4, 8, 12, 16, 20]
                for id in tipIds:
                    cv2.circle(img, (lmList[id][1], lmList[id][2]), 8, (255, 0, 0), cv2.FILLED)
            
            # Calculate and display FPS
            cTime = time.time()
            fps = 1/(cTime-pTime)
            pTime = cTime
            
            # Display FPS and command
            cv2.putText(img, f'FPS: {int(fps)}', (10,30), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
            cv2.putText(img, f'Command: {command_text}', (10,70), cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
            
            # Show the image
            cv2.imshow("Hand Tracking", img)
            
            # Break loop on 'q' press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cap.release()
        cv2.destroyAllWindows()
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()