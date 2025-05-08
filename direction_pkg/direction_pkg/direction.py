import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import torch
import numpy as np
from geometry_msgs.msg import Twist  # For more advanced motion control
import cv2  # For image processing
from sensor_msgs.msg import Image  # For subscribing to image data

class DirectionPublisher(Node):
    def __init__(self):
        super().__init__('direction_publisher')
        # Publisher for simple direction commands (left/right)
        self.direction_publisher = self.create_publisher(String, 'direction', 10)
        # Publisher for radial grade (-1.0 to 1.0, where -1 is full left, 1 is full right, 0 is straight)
        self.steering_publisher = self.create_publisher(Float32, 'steering_angle', 10)
        # Optional: Publisher for velocity commands if you want to control speed too
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscriber to camera image (if available)
        self.camera_subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        
        # Store the latest image for processing
        self.latest_image = None
        
        timer_period = 0.1  # seconds (increased frequency for more responsive control)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Load PyTorch segmentation model
        self.model = self.load_model()
        self.get_logger().info('Model loaded successfully!')
        
        # Parameters for obstacle avoidance and path following
        self.declare_parameter('min_safe_distance', 1.0)
        self.declare_parameter('max_steering_angle', 0.8)  # in radians
        self.declare_parameter('turn_around_threshold', 0.1)  # Minimum visible path area to trigger a turn around
        self.declare_parameter('path_class_id', 1)  # ID of the walkable path in segmentation mask
        
        # State variables
        self.is_turning_around = False
        self.turn_around_start_time = None
        self.turn_around_duration = 3.0  # seconds to complete 180-degree turn

    def load_model(self):
        try:
            # Replace with your actual model path and loading code
            model_path = self.get_parameter('model_path').value if self.has_parameter('model_path') else None
            if model_path is None:
                model_path = 'path_to_your_model.pth'  # Default path
            
            # Check if GPU is available
            device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            self.get_logger().info(f'Using device: {device}')
            
            # Load model to the detected device
            model = torch.load(model_path, map_location=device)
            model.eval()
            model.to(device)  # Move model to GPU if available
            
            # Store the device for later use with tensors
            self.device = device
            
            return model
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {str(e)}')
            # Return a dummy model for testing purposes
            self.device = torch.device('cpu')
            return None
    
    def image_callback(self, msg):
        """Callback function for the camera image subscription"""
        self.latest_image = msg
        
    def preprocess_image(self, image_data):
        """Convert ROS Image to PyTorch tensor for model input"""
        if image_data is None:
            # Generate dummy data for testing if no image is available
            return torch.randn(1, 3, 224, 224, device=self.device)
            
        try:
            # Convert ROS Image message to OpenCV image
            import cv_bridge
            bridge = cv_bridge.CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image_data, desired_encoding='rgb8')
            
            # Resize to match model's expected input size
            cv_image = cv2.resize(cv_image, (224, 224))
            
            # Convert to PyTorch tensor
            tensor = torch.from_numpy(cv_image.transpose(2, 0, 1)).float().unsqueeze(0) / 255.0
            # Move tensor to the same device as the model
            tensor = tensor.to(self.device)
            return tensor
        except Exception as e:
            self.get_logger().error(f'Failed to preprocess image: {str(e)}')
            return torch.randn(1, 3, 224, 224, device=self.device)  # Fallback to random data

    def process_segmentation(self, segmentation_output):
        """
        Process segmentation output to determine:
        1. Path location and center
        2. Check for dead ends
        3. Optimal steering direction and angle
        """
        # Assuming segmentation_output is a tensor where each value represents class probabilities
        # Get the class with highest probability (argmax) for each pixel
        if len(segmentation_output.shape) == 4 and segmentation_output.shape[1] > 1:
            # Multi-class segmentation (B, C, H, W)
            segmentation_mask = torch.argmax(segmentation_output, dim=1).squeeze(0)
        else:
            # Binary segmentation
            segmentation_mask = (segmentation_output > 0.5).squeeze().int()
        
        # Extract path from segmentation mask (using the path class ID)
        path_class_id = self.get_parameter('path_class_id').value
        path_mask = (segmentation_mask == path_class_id).float()
        
        # Calculate the ratio of path pixels in the image
        path_ratio = torch.mean(path_mask).item()
        
        # Check for dead end (not enough path visible)
        turn_around_threshold = self.get_parameter('turn_around_threshold').value
        dead_end_detected = path_ratio < turn_around_threshold
        
        if dead_end_detected and not self.is_turning_around:
            # Initiate turn around sequence
            self.get_logger().info('Dead end detected! Initiating 180-degree turn.')
            self.is_turning_around = True
            self.turn_around_start_time = self.get_clock().now()
            return "umdrehen", -1.0  # "turn around" in German, full left turn
        
        if self.is_turning_around:
            # Continue turning until turn duration has elapsed
            time_elapsed = (self.get_clock().now() - self.turn_around_start_time).nanoseconds / 1e9
            if time_elapsed < self.turn_around_duration:
                return "umdrehen", -1.0  # Continue turning
            else:
                self.is_turning_around = False
                self.get_logger().info('Turn around completed.')
        
        # Calculate the centroid of the path
        height, width = path_mask.shape
        
        # Create column and row indices
        col_indices = torch.arange(0, width, device=path_mask.device)
        row_indices = torch.arange(0, height, device=path_mask.device)
        
        # Calculate weighted centroid
        total_mass = torch.sum(path_mask)
        if total_mass > 0:
            # Calculate center of mass for columns (x coordinate)
            col_matrix = col_indices.unsqueeze(0).repeat(height, 1)
            x_center = torch.sum(col_matrix * path_mask) / total_mass
            
            # Map x_center to steering angle
            # Center of image (width/2) should give 0.0 steering angle
            # Left edge (0) should give max negative steering
            # Right edge (width) should give max positive steering
            normalized_x_center = (x_center / width) - 0.5  # Range: -0.5 to 0.5
            max_angle = self.get_parameter('max_steering_angle').value
            steering_angle = max_angle * (normalized_x_center * 2.0)  # Scale to -max_angle to max_angle
            
            # Determine direction category
            if steering_angle < -0.2:
                direction = "links"  # left
            elif steering_angle > 0.2:
                direction = "rechts"  # right
            else:
                direction = "geradeaus"  # straight ahead
                
            return direction, steering_angle
        else:
            # No path detected at all, default to turn around
            return "umdrehen", -1.0

    def get_model_prediction(self, input_data):
        if self.model is None:
            return "geradeaus", 0.0
            
        # Process input through model
        with torch.no_grad():
            segmentation_output = self.model(input_data)
            direction, steering_angle = self.process_segmentation(segmentation_output)
        
        return direction, steering_angle

    def timer_callback(self):
        try:
            # Get image data from camera or generate test data
            input_data = self.preprocess_image(self.latest_image)
            
            # Get model prediction
            direction, steering_angle = self.get_model_prediction(input_data)
            
            # Publish direction as string
            direction_msg = String()
            direction_msg.data = direction
            self.direction_publisher.publish(direction_msg)
            
            # Publish steering angle as float
            steering_msg = Float32()
            steering_msg.data = float(steering_angle)
            self.steering_publisher.publish(steering_msg)
            
            # Adjust linear velocity based on maneuvers
            linear_velocity = 0.0 if direction == "umdrehen" else 0.2
            
            # Optionally publish Twist for direct robot control
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = steering_angle
            self.cmd_vel_publisher.publish(twist_msg)
            
            self.get_logger().info(f'Direction: {direction}, Steering: {steering_angle:.2f}')
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    direction_publisher = DirectionPublisher()
    rclpy.spin(direction_publisher)
    direction_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
