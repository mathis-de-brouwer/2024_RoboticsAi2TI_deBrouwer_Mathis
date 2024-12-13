import cv2
import mediapipe as mp
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

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
        
        # For thumb - check if it's on right side of hand
        if lmList[tipIds[0]][1] > lmList[tipIds[0] - 1][1]:
            fingers.append(1)
        else:
            fingers.append(0)
            
        # For other 4 fingers
        for id in range(1, 5):
            if lmList[tipIds[id]][2] < lmList[tipIds[id] - 2][2]:
                fingers.append(1)
            else:
                fingers.append(0)
        
        total = sum(fingers)
        return total

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

def main(args=None):
    rclpy.init(args=args)
    
    pTime = 0
    cTime = 0
    
    cap = cv2.VideoCapture(0)
    detector = handDetector()
    
    while rclpy.ok():
        success, img = cap.read()
        if not success:
            continue
            
        # Flip the image horizontally for a later selfie-view display
        img = cv2.flip(img, 1)
        img = detector.findHands(img)
        lmList = detector.findPosition(img)
        
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
            
            # Display finger count and command
            cv2.putText(img, f'Fingers: {fingers_up} - {command_text}', (10,70), 
                       cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
        else:
            # Display no hand detected
            cv2.putText(img, command_text, (10,70), 
                       cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
        
        cTime = time.time()
        fps = 1/(cTime-pTime)
        pTime = cTime
        
        # Display FPS
        cv2.putText(img, f'FPS: {int(fps)}', (10,30), 
                   cv2.FONT_HERSHEY_PLAIN, 2, (255,0,255), 2)
        
        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
