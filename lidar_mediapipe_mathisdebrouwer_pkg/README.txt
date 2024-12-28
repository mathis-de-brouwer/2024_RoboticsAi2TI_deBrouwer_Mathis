Hand Gesture Controlled TurtleBot with Obstacle Avoidance
=======================================================

Requirements:
------------
- Ubuntu 22.04.5 LTS
- ROS 2 Humble
- Python 3.10.12
- OpenCV
- MediaPipe
- Webcam
- TurtleBot3 (real or simulated)

Installation:
------------
1. Make sure you have all dependencies installed:
   ```
   pip3 install opencv-python mediapipe numpy
   ```

2. Copy the package to your ROS 2 workspace:
   ```
   cd ~/ros2_ws/src
   git clone [your-repository-url]
   ```

3. Build the package:
   ```
   cd ~/ros2_ws
   colcon build --packages-select lidar_mediapipe_mathisdebrouwer_pkg
   source install/setup.bash
   ```

Running the Program:
------------------
1. First, start your TurtleBot3 (choose one):

   a. For Simulation: from ros2_ws directory after sourcing the setup.bash file
      ```
      export TURTLEBOT3_MODEL=burger
      ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
      ```
   
   b. For Real TurtleBot3:
      - Power on your TurtleBot3
      - Ensure your computer is connected to the TurtleBot3's network
      - Set the correct ROS_DOMAIN_ID

2. Launch the hand gesture control program: from ros2_ws directory after sourcing the setup.bash file
   ```
   ros2 launch lidar_mediapipe_mathisdebrouwer_pkg lidar_mediapipe_launch.launch.py
   ```

Hand Gesture Controls:
-------------------
- 5 fingers: Move Forward
- 0 fingers (fist): Stop
- 1 finger: Turn Left
- 3 fingers: Turn Right

Notes:
-----
- The program will automatically avoid obstacles while following hand gesture commands
- Press 'q' in the camera window to quit the program
- The robot will stop automatically if:
  * No hand gesture is detected
  * An obstacle is detected too close
  * LiDAR data is invalid

Troubleshooting:
--------------
1. If camera doesn't open:
   - Check if your webcam is connected
   - Try a different video device: modify '0' in cv2.VideoCapture(0)

2. If robot doesn't move:
   - Check if TurtleBot3/simulator is running
   - Verify ROS topics: ros2 topic list
   - Check ROS_DOMAIN_ID if using real robot

3. If obstacle avoidance doesn't work:
   - Verify LiDAR data: ros2 topic echo /scan
   - Check if the robot receives commands: ros2 topic echo /cmd_vel 