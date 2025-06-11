# ROS2 Jazzy Jalisco Implementation Guide
## Complete CR3 Hand Tracking and Robot Control System

### Project Overview
This guide provides a complete ROS2 Jazzy Jalisco implementation for CR3 robot control with hand tracking. It combines the lessons learned from the failed Hand_Tracking project with modern ROS2 architecture.

### Architecture
```
Camera → Python Hand Tracking Node → ROS2 Topics → C++ Robot Control Node → TCP-IP-ROS-6Axis → CR3 Robot
```

### Advantages of ROS2 Jazzy over ROS1
- **Modern QoS Profiles**: Better reliability and performance control
- **Built-in Security**: DDS security features
- **Cross-platform Support**: Better Windows/Linux compatibility
- **Improved Communication**: More efficient message passing
- **Python 3.11+ Support**: Latest Python features and libraries
- **Component Architecture**: Better modularity and lifecycle management

---

## 1. Package Structure

```
cr3_hand_control/
├── package.xml
├── CMakeLists.txt
├── setup.py
├── src/
│   └── cr3_hand_control/
│       ├── __init__.py
│       ├── hand_tracking_node.py
│       └── robot_control_node.cpp
├── msg/
│   ├── HandCoordinates.msg
│   └── RobotStatus.msg
├── launch/
│   ├── cr3_complete_system.launch.py
│   ├── hand_tracking_only.launch.py
│   └── robot_control_only.launch.py
├── config/
│   ├── hand_tracking_params.yaml
│   └── robot_control_params.yaml
├── scripts/
│   └── install_dependencies.sh
└── README.md
```

---

## 2. Custom Message Definitions

### HandCoordinates.msg
```
# Custom message for hand tracking coordinates
std_msgs/Header header
geometry_msgs/Point position
float32 confidence
bool hand_detected
string landmark_type  # "index_tip", "palm_center", etc.
```

### RobotStatus.msg
```
# Robot status information
std_msgs/Header header
bool robot_connected
bool robot_enabled
bool moving_to_target
geometry_msgs/Point current_position
geometry_msgs/Point target_position
string status_message
```

---

## 3. Python Hand Tracking Node

### hand_tracking_node.py
    def __init__(self):
        super().__init__('hand_tracking_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('detection_confidence', 0.7)
        self.declare_parameter('tracking_confidence', 0.5)
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.tracking_confidence = self.get_parameter('tracking_confidence').value
        
        # QoS Profiles for different data types
        self.coordinate_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.hand_coord_pub = self.create_publisher(
            PointStamped, 
            'hand_coordinates', 
            self.coordinate_qos
        )
        
        self.tracking_status_pub = self.create_publisher(
            Bool, 
            'tracking_active', 
            self.coordinate_qos
        )
        
        self.debug_image_pub = self.create_publisher(
            Image, 
            'debug_image', 
            self.image_qos
        )
        
        # Subscribers (for robot status feedback)
        self.robot_status_sub = self.create_subscription(
            Bool,
            'robot_enabled',
            self.robot_status_callback,
            self.coordinate_qos
        )
        
        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=self.detection_confidence,
            min_tracking_confidence=self.tracking_confidence
        )
        self.mp_drawing = mp.solutions.drawing_utils
        
        # OpenCV setup
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # CV Bridge for ROS image messages
        self.bridge = CvBridge()
        
        # State variables
        self.robot_enabled = False
        self.last_hand_position = None
        
        # Timer for main processing loop
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.process_frame
        )
        
        self.get_logger().info(f'Hand Tracking Node initialized - Publishing at {self.publish_rate} Hz')
    
    def robot_status_callback(self, msg):
        """Handle robot enable/disable status"""
        self.robot_enabled = msg.data
        if self.robot_enabled:
            self.get_logger().info('Robot enabled - Hand tracking active')
        else:
            self.get_logger().info('Robot disabled - Hand tracking paused')
    
    def process_frame(self):
        """Main processing loop - capture frame and detect hands"""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return
        
        # Flip frame horizontally for mirror effect
        frame = cv2.flip(frame, 1)
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb_frame)
        
        # Create timestamp
        timestamp = self.get_clock().now().to_msg()
        
        hand_detected = False
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Get index finger tip (landmark 8)
                index_tip = hand_landmarks.landmark[8]
                
                # Convert normalized coordinates to pixel coordinates
                h, w, c = frame.shape
                pixel_x = int(index_tip.x * w)
                pixel_y = int(index_tip.y * h)
                
                # Convert to robot coordinate system
                # This transformation depends on your camera setup and robot workspace
                robot_coords = self.pixel_to_robot_coordinates(
                    index_tip.x, index_tip.y, index_tip.z
                )
                
                # Create and publish coordinate message
                coord_msg = PointStamped()
                coord_msg.header = Header()
                coord_msg.header.stamp = timestamp
                coord_msg.header.frame_id = 'camera_frame'
                coord_msg.point.x = robot_coords[0]
                coord_msg.point.y = robot_coords[1]
                coord_msg.point.z = robot_coords[2]
                
                self.hand_coord_pub.publish(coord_msg)
                
                # Draw landmarks on debug image
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS
                )
                
                # Draw coordinate info
                cv2.putText(frame, 
                           f'Robot: ({robot_coords[0]:.2f}, {robot_coords[1]:.2f}, {robot_coords[2]:.2f})',
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Highlight index finger tip
                cv2.circle(frame, (pixel_x, pixel_y), 10, (255, 0, 0), -1)
                
                hand_detected = True
                self.last_hand_position = robot_coords
                break
        
        # Publish tracking status
        status_msg = Bool()
        status_msg.data = hand_detected and self.robot_enabled
        self.tracking_status_pub.publish(status_msg)
        
        # Add status overlay to debug image
        status_text = "TRACKING ACTIVE" if (hand_detected and self.robot_enabled) else "NO TRACKING"
        status_color = (0, 255, 0) if (hand_detected and self.robot_enabled) else (0, 0, 255)
        cv2.putText(frame, status_text, (10, frame.shape[0] - 20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        
        robot_status = "ROBOT ENABLED" if self.robot_enabled else "ROBOT DISABLED"
        cv2.putText(frame, robot_status, (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Publish debug image
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            debug_msg.header.stamp = timestamp
            debug_msg.header.frame_id = 'camera_frame'
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')
    
    def pixel_to_robot_coordinates(self, norm_x, norm_y, norm_z):
        """
        Convert normalized MediaPipe coordinates to robot workspace coordinates
        
        Args:
            norm_x, norm_y, norm_z: Normalized coordinates (0-1) from MediaPipe
            
        Returns:
            tuple: (x, y, z) coordinates in robot workspace (mm)
        """
        # Define your camera-to-robot transformation here
        # This is specific to your camera setup and robot workspace
        
        # Example transformation (adjust for your setup):
        # Camera view maps to robot workspace
        workspace_width = 400   # mm
        workspace_height = 300  # mm
        workspace_depth = 200   # mm
        
        # Offset from robot base
        base_x = 200  # mm forward from robot base
        base_y = 0    # mm centered
        base_z = 150  # mm above table
        
        # Transform coordinates
        robot_x = base_x + (norm_x - 0.5) * workspace_width
        robot_y = base_y + (0.5 - norm_y) * workspace_height  # Flip Y axis
        robot_z = base_z + norm_z * workspace_depth
        
        return (robot_x, robot_y, robot_z)
    
    def destroy_node(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = HandTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
