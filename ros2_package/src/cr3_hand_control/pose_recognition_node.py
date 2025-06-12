#!/usr/bin/env python3
"""
ROS2 Pose Recognition Node

Based on the working implementation from Dashboards/hand_tracking/Hand_Tracking.py
Provides shoulder and wrist tracking with ROS2 integration for robot control.

This node combines MediaPipe pose and hand detection for accurate tracking
of shoulder and wrist positions, publishing coordinates for robot control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import custom messages
from cr3_hand_control.msg import PoseCoordinates, PoseTrackingStatus, DebugInfo

import cv2
import mediapipe as mp
import numpy as np
import time
import threading


class PoseRecognitionNode(Node):
    """ROS2 node for pose recognition using MediaPipe and OpenCV"""
    
    def __init__(self):
        super().__init__('pose_recognition_node')
        
        # Declare parameters
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('pose_detection_confidence', 0.5)
        self.declare_parameter('pose_tracking_confidence', 0.5)
        self.declare_parameter('hand_detection_confidence', 0.5)
        self.declare_parameter('hand_tracking_confidence', 0.5)
        self.declare_parameter('tracked_hand', 'Right')
        self.declare_parameter('enable_debug_image', True)
        self.declare_parameter('enable_robot_control', True)
        
        # Get parameters
        self.camera_id = self.get_parameter('camera_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.pose_detection_confidence = self.get_parameter('pose_detection_confidence').value
        self.pose_tracking_confidence = self.get_parameter('pose_tracking_confidence').value
        self.hand_detection_confidence = self.get_parameter('hand_detection_confidence').value
        self.hand_tracking_confidence = self.get_parameter('hand_tracking_confidence').value
        self.tracked_hand = self.get_parameter('tracked_hand').value
        self.enable_debug_image = self.get_parameter('enable_debug_image').value
        self.enable_robot_control = self.get_parameter('enable_robot_control').value
        
        # QoS Profiles
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
        self.pose_coord_pub = self.create_publisher(
            PoseCoordinates, 
            'pose_coordinates', 
            self.coordinate_qos
        )
        
        self.tracking_status_pub = self.create_publisher(
            PoseTrackingStatus, 
            'pose_tracking_status', 
            self.coordinate_qos
        )
        
        self.debug_info_pub = self.create_publisher(
            DebugInfo,
            'pose_debug_info',
            self.coordinate_qos
        )
        
        if self.enable_debug_image:
            self.debug_image_pub = self.create_publisher(
                Image, 
                'pose_debug_image', 
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
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        
        # Initialize MediaPipe models
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=self.hand_detection_confidence,
            min_tracking_confidence=self.hand_tracking_confidence
        )
        
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=self.pose_detection_confidence,
            min_tracking_confidence=self.pose_tracking_confidence
        )
        
        # OpenCV setup
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        
        # CV Bridge for ROS image messages
        self.bridge = CvBridge()
        
        # State variables
        self.robot_enabled = self.enable_robot_control
        self.last_shoulder_position = None
        self.last_wrist_position = None
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        self.last_frame_time = time.time()
        self.processing_times = []
        self.pose_detection_count = 0
        self.hand_detection_count = 0
        
        # Timer for main processing loop
        self.timer = self.create_timer(
            1.0 / self.publish_rate, 
            self.process_frame
        )
        
        self.get_logger().info(f'Pose Recognition Node initialized')
        self.get_logger().info(f'Tracking: {self.tracked_hand} hand')
        self.get_logger().info(f'Publishing at {self.publish_rate} Hz')
        self.get_logger().info(f'Robot control: {"Enabled" if self.robot_enabled else "Disabled"}')
    
    def robot_status_callback(self, msg):
        """Handle robot enable/disable status"""
        self.robot_enabled = msg.data
        if self.robot_enabled:
            self.get_logger().info('Robot enabled - Pose tracking active')
        else:
            self.get_logger().info('Robot disabled - Pose tracking paused')
    
    def process_frame(self):
        """Main processing loop - capture frame and detect pose/hands"""
        start_process_time = time.time()
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return
        
        # Convert BGR to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process frame with both MediaPipe models
        pose_start_time = time.time()
        pose_results = self.pose.process(rgb_frame)
        pose_process_time = time.time() - pose_start_time
        
        hand_start_time = time.time()
        hand_results = self.hands.process(rgb_frame)
        hand_process_time = time.time() - hand_start_time
        
        # Create timestamp
        timestamp = self.get_clock().now().to_msg()
        
        # Variables to store landmarks
        right_shoulder = None
        right_wrist = None
        pose_detected = False
        hand_detected = False
        pose_confidence = 0.0
        hand_confidence = 0.0
        
        # Create overlay for visualization
        if self.enable_debug_image:
            overlay = frame.copy()
        
        # Extract shoulder position from pose
        if pose_results.pose_landmarks:
            pose_detected = True
            self.pose_detection_count += 1
            
            # Get shoulder position (right shoulder landmark)
            shoulder_landmark = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            right_shoulder = [shoulder_landmark.x, shoulder_landmark.y, shoulder_landmark.z]
            pose_confidence = shoulder_landmark.visibility if hasattr(shoulder_landmark, 'visibility') else 0.8
            
            # Fallback wrist position from pose (in case hand detection fails)
            wrist_landmark = pose_results.pose_landmarks.landmark[self.mp_pose.PoseLandmark.RIGHT_WRIST]
            right_wrist = [wrist_landmark.x, wrist_landmark.y, wrist_landmark.z]
            
            if self.enable_debug_image:
                # Draw pose landmarks
                self.mp_drawing.draw_landmarks(
                    overlay,
                    pose_results.pose_landmarks,
                    self.mp_pose.POSE_CONNECTIONS,
                    landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=4, circle_radius=6),
                    connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2)
                )
                
                # Highlight shoulder
                shoulder_pixel = (
                    int(right_shoulder[0] * frame.shape[1]), 
                    int(right_shoulder[1] * frame.shape[0])
                )
                cv2.circle(overlay, shoulder_pixel, 10, (0, 0, 255), -1)
                cv2.putText(overlay, "SHOULDER", (shoulder_pixel[0]+10, shoulder_pixel[1]), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Extract more precise wrist position from hand tracking
        if hand_results.multi_hand_landmarks and hand_results.multi_handedness:
            for hand_landmarks, hand_info in zip(hand_results.multi_hand_landmarks, hand_results.multi_handedness):
                label = hand_info.classification[0].label
                # Note: MediaPipe flips the hand labels for camera view
                label = "Left" if label == "Right" else "Right"
                
                if label == self.tracked_hand:
                    hand_detected = True
                    self.hand_detection_count += 1
                    hand_confidence = hand_info.classification[0].score
                    
                    # Get precise wrist position from hand landmarks
                    wrist_landmark = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                    right_wrist = [wrist_landmark.x, wrist_landmark.y, wrist_landmark.z]
                    
                    if self.enable_debug_image:
                        # Draw hand landmarks
                        self.mp_drawing.draw_landmarks(
                            overlay,
                            hand_landmarks,
                            self.mp_hands.HAND_CONNECTIONS,
                            landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=5, circle_radius=8),
                            connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=3)
                        )
                        
                        # Highlight wrist
                        wrist_pixel = (
                            int(right_wrist[0] * frame.shape[1]),
                            int(right_wrist[1] * frame.shape[0])
                        )
                        cv2.circle(overlay, wrist_pixel, 10, (255, 0, 0), -1)
                        cv2.putText(overlay, "WRIST", (wrist_pixel[0]+10, wrist_pixel[1]),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    break
        
        # Draw connection line between shoulder and wrist
        if self.enable_debug_image and right_shoulder is not None and right_wrist is not None:
            shoulder_px = (int(right_shoulder[0] * frame.shape[1]), int(right_shoulder[1] * frame.shape[0]))
            wrist_px = (int(right_wrist[0] * frame.shape[1]), int(right_wrist[1] * frame.shape[0]))
            cv2.line(overlay, shoulder_px, wrist_px, (255, 0, 255), 3)
            
            # Blend overlay with original frame
            alpha = 0.7
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        
        # Create and publish pose coordinates message
        if right_shoulder is not None and right_wrist is not None:
            pose_msg = PoseCoordinates()
            pose_msg.header = Header()
            pose_msg.header.stamp = timestamp
            pose_msg.header.frame_id = 'camera_frame'
            
            # Convert normalized coordinates to robot workspace coordinates
            shoulder_robot_coords = self.normalize_to_robot_coordinates(right_shoulder)
            wrist_robot_coords = self.normalize_to_robot_coordinates(right_wrist)
            
            pose_msg.shoulder_position = Point(x=shoulder_robot_coords[0], y=shoulder_robot_coords[1], z=shoulder_robot_coords[2])
            pose_msg.shoulder_confidence = pose_confidence
            pose_msg.wrist_position = Point(x=wrist_robot_coords[0], y=wrist_robot_coords[1], z=wrist_robot_coords[2])
            pose_msg.wrist_confidence = hand_confidence
            pose_msg.pose_detected = pose_detected
            pose_msg.hand_detected = hand_detected
            pose_msg.tracked_hand = self.tracked_hand
            
            self.pose_coord_pub.publish(pose_msg)
            
            # Store last positions
            self.last_shoulder_position = shoulder_robot_coords
            self.last_wrist_position = wrist_robot_coords
        
        # Calculate performance metrics
        current_time = time.time()
        frame_time = current_time - self.last_frame_time
        self.last_frame_time = current_time
        total_process_time = current_time - start_process_time
        self.processing_times.append(total_process_time)
        if len(self.processing_times) > 30:  # Keep last 30 frames
            self.processing_times.pop(0)
        
        self.frame_count += 1
        
        # Publish tracking status
        status_msg = PoseTrackingStatus()
        status_msg.header = Header()
        status_msg.header.stamp = timestamp
        status_msg.header.frame_id = 'camera_frame'
        status_msg.tracking_active = pose_detected and self.robot_enabled
        status_msg.pose_detected = pose_detected
        status_msg.hand_detected = hand_detected
        status_msg.pose_confidence = pose_confidence
        status_msg.hand_confidence = hand_confidence
        status_msg.fps = 1.0 / frame_time if frame_time > 0 else 0.0
        status_msg.frame_count = self.frame_count
        status_msg.processing_time_ms = total_process_time * 1000
        status_msg.robot_enabled = self.robot_enabled
        status_msg.robot_connected = True  # Assume connected for now
        status_msg.status_message = "Active tracking" if status_msg.tracking_active else "Tracking paused"
        
        self.tracking_status_pub.publish(status_msg)
        
        # Publish debug info every 30 frames
        if self.frame_count % 30 == 0:
            self.publish_debug_info(timestamp, frame_time, pose_process_time, hand_process_time, total_process_time)
        
        # Publish debug image
        if self.enable_debug_image:
            self.publish_debug_image(frame, timestamp)
    
    def normalize_to_robot_coordinates(self, normalized_coords):
        """
        Convert normalized MediaPipe coordinates to robot workspace coordinates
        
        Args:
            normalized_coords: [x, y, z] normalized coordinates (0-1) from MediaPipe
            
        Returns:
            tuple: (x, y, z) coordinates in robot workspace (mm)
        """
        # Define camera-to-robot transformation
        # This should match your camera setup and robot workspace
        workspace_width = 400   # mm
        workspace_height = 300  # mm
        workspace_depth = 200   # mm
        
        # Offset from robot base
        base_x = 200  # mm forward from robot base
        base_y = 0    # mm centered
        base_z = 150  # mm above table
        
        # Transform coordinates
        robot_x = base_x + (normalized_coords[0] - 0.5) * workspace_width
        robot_y = base_y + (0.5 - normalized_coords[1]) * workspace_height  # Flip Y axis
        robot_z = base_z + normalized_coords[2] * workspace_depth
        
        return (robot_x, robot_y, robot_z)
    
    def publish_debug_info(self, timestamp, frame_time, pose_time, hand_time, total_time):
        """Publish debug information"""
        debug_msg = DebugInfo()
        debug_msg.header = Header()
        debug_msg.header.stamp = timestamp
        debug_msg.header.frame_id = 'camera_frame'
        
        # Performance metrics
        debug_msg.camera_fps = 1.0 / frame_time if frame_time > 0 else 0.0
        debug_msg.mediapipe_fps = 1.0 / total_time if total_time > 0 else 0.0
        debug_msg.total_processing_time = total_time * 1000  # ms
        debug_msg.pose_processing_time = pose_time * 1000    # ms
        debug_msg.hand_processing_time = hand_time * 1000    # ms
        
        # Detection statistics
        debug_msg.total_frames_processed = self.frame_count
        debug_msg.frames_with_pose = self.pose_detection_count
        debug_msg.frames_with_hand = self.hand_detection_count
        debug_msg.pose_detection_rate = self.pose_detection_count / self.frame_count if self.frame_count > 0 else 0.0
        debug_msg.hand_detection_rate = self.hand_detection_count / self.frame_count if self.frame_count > 0 else 0.0
        
        # System status
        debug_msg.camera_status = "Active"
        debug_msg.mediapipe_status = "Active"
        debug_msg.error_messages = []
        debug_msg.warning_messages = []
        
        self.debug_info_pub.publish(debug_msg)
        
        # Log performance info
        avg_processing_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0
        self.get_logger().info(f'Performance: {debug_msg.camera_fps:.1f} FPS, '
                              f'Avg processing: {avg_processing_time*1000:.1f}ms, '
                              f'Pose detection: {debug_msg.pose_detection_rate:.2f}, '
                              f'Hand detection: {debug_msg.hand_detection_rate:.2f}')
    
    def publish_debug_image(self, frame, timestamp):
        """Publish debug image with overlays"""
        try:
            # Add status overlays
            status_text = "TRACKING ACTIVE" if self.robot_enabled else "TRACKING PAUSED"
            status_color = (0, 255, 0) if self.robot_enabled else (0, 0, 255)
            cv2.putText(frame, status_text, (10, frame.shape[0] - 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
            
            robot_status = f"ROBOT: {'ENABLED' if self.robot_enabled else 'DISABLED'}"
            cv2.putText(frame, robot_status, (10, frame.shape[0] - 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # Add coordinate info if available
            if self.last_shoulder_position and self.last_wrist_position:
                coord_text = f"S: ({self.last_shoulder_position[0]:.0f}, {self.last_shoulder_position[1]:.0f}, {self.last_shoulder_position[2]:.0f})"
                cv2.putText(frame, coord_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                coord_text = f"W: ({self.last_wrist_position[0]:.0f}, {self.last_wrist_position[1]:.0f}, {self.last_wrist_position[2]:.0f})"
                cv2.putText(frame, coord_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            debug_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            debug_msg.header.stamp = timestamp
            debug_msg.header.frame_id = 'camera_frame'
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().warn(f'Failed to publish debug image: {e}')
    
    def destroy_node(self):
        """Clean up resources"""
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = PoseRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
