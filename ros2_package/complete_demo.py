#!/usr/bin/env python3
"""
Complete CR3 Pose Recognition System Demo
Demonstrates the full integration of visual tracking with ROS2 messaging
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import custom messages
from cr3_hand_control.msg import PoseCoordinates, PoseTrackingStatus, DebugInfo

import cv2
import mediapipe as mp
import numpy as np
import time
import threading
import sys
import os

# Add path for UI components
sys.path.append('/home/andrewlh/CR3_OpenCV_Controls/Dashboards/hand_tracking')
from ui_components import create_ui_elements, create_mouse_callback

class CompletePoseRecognitionDemo(Node):
    """Complete demo showing both visual interface and ROS2 integration"""
    
    def __init__(self):
        super().__init__('complete_pose_demo')
        
        # ROS2 Setup
        self.coordinate_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
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
        
        # MediaPipe setup
        self.mp_hands = mp.solutions.hands
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # State variables
        self.running_state = {
            'running': True,
            'paused': False,
            'mirrored': False,
            'robot_enabled': True
        }
        
        # Performance tracking
        self.frame_count = 0
        self.pose_detections = 0
        self.hand_detections = 0
        self.start_time = time.time()
        
        # Coordinate storage
        self.right_shoulder_coords = None
        self.right_wrist_coords = None
        
        self.get_logger().info('🚀 Complete Pose Recognition Demo Started!')
        self.get_logger().info('📱 Visual dashboard with ROS2 integration active')
    
    def normalize_to_robot_coordinates(self, normalized_coords):
        """Convert normalized coordinates to robot workspace coordinates"""
        workspace_width = 400   # mm
        workspace_height = 300  # mm
        workspace_depth = 200   # mm
        
        base_x = 200  # mm forward from robot base
        base_y = 0    # mm centered
        base_z = 150  # mm above table
        
        robot_x = base_x + (normalized_coords[0] - 0.5) * workspace_width
        robot_y = base_y + (0.5 - normalized_coords[1]) * workspace_height
        robot_z = base_z + normalized_coords[2] * workspace_depth
        
        return (robot_x, robot_y, robot_z)
    
    def process_frame(self):
        """Process a single frame with pose detection and UI"""
        ret, frame = self.cap.read()
        if not ret:
            return None, None, None
        
        if self.running_state['mirrored']:
            frame = cv2.flip(frame, 1)
        
        if self.running_state['paused']:
            # Show paused message
            cv2.putText(frame, "PAUSED", (frame.shape[1]//2 - 100, frame.shape[0]//2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 165, 255), 3)
            return frame, None, None
        
        # Convert and process
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pose_results = self.pose.process(rgb_frame)
        hand_results = self.hands.process(rgb_frame)
        
        # Variables for tracking
        right_shoulder = None
        right_wrist = None
        pose_detected = False
        hand_detected = False
        
        self.frame_count += 1
        
        # Process pose results
        if pose_results.pose_landmarks:
            pose_detected = True
            self.pose_detections += 1
            
            # Draw pose landmarks
            self.mp_drawing.draw_landmarks(
                frame,
                pose_results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=3, circle_radius=5),
                connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2)
            )
            
            # Get shoulder and wrist positions
            landmarks = pose_results.pose_landmarks.landmark
            shoulder_landmark = landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER]
            wrist_landmark = landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST]
            
            right_shoulder = [shoulder_landmark.x, shoulder_landmark.y, shoulder_landmark.z]
            right_wrist = [wrist_landmark.x, wrist_landmark.y, wrist_landmark.z]
            
            # Convert to robot coordinates
            self.right_shoulder_coords = self.normalize_to_robot_coordinates(right_shoulder)
            self.right_wrist_coords = self.normalize_to_robot_coordinates(right_wrist)
            
            # Highlight key points
            h, w = frame.shape[:2]
            shoulder_px = (int(shoulder_landmark.x * w), int(shoulder_landmark.y * h))
            wrist_px = (int(wrist_landmark.x * w), int(wrist_landmark.y * h))
            
            cv2.circle(frame, shoulder_px, 10, (0, 0, 255), -1)
            cv2.circle(frame, wrist_px, 10, (255, 0, 0), -1)
            cv2.line(frame, shoulder_px, wrist_px, (255, 0, 255), 3)
            
            cv2.putText(frame, "SHOULDER", (shoulder_px[0]+15, shoulder_px[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.putText(frame, "WRIST", (wrist_px[0]+15, wrist_px[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        # Process hand results for more precise wrist tracking
        if hand_results.multi_hand_landmarks and hand_results.multi_handedness:
            for hand_landmarks, hand_info in zip(hand_results.multi_hand_landmarks, hand_results.multi_handedness):
                label = hand_info.classification[0].label
                label = "Left" if label == "Right" else "Right"  # MediaPipe flips labels
                
                if label == "Right":
                    hand_detected = True
                    self.hand_detections += 1
                    
                    # Draw hand landmarks
                    self.mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                        landmark_drawing_spec=self.mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=4, circle_radius=6),
                        connection_drawing_spec=self.mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2)
                    )
                    
                    # Update wrist position with more precise hand tracking
                    wrist_landmark = hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST]
                    right_wrist = [wrist_landmark.x, wrist_landmark.y, wrist_landmark.z]
                    self.right_wrist_coords = self.normalize_to_robot_coordinates(right_wrist)
                    break
        
        # Publish ROS2 messages if we have valid data
        if self.right_shoulder_coords and self.right_wrist_coords and self.running_state['robot_enabled']:
            self.publish_pose_data(pose_detected, hand_detected)
        
        return frame, self.right_shoulder_coords, self.right_wrist_coords
    
    def publish_pose_data(self, pose_detected, hand_detected):
        """Publish pose data to ROS2 topics"""
        timestamp = self.get_clock().now().to_msg()
        
        # Publish coordinates
        pose_msg = PoseCoordinates()
        pose_msg.header = Header()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'camera_frame'
        
        pose_msg.shoulder_position = Point(
            x=self.right_shoulder_coords[0], 
            y=self.right_shoulder_coords[1], 
            z=self.right_shoulder_coords[2]
        )
        pose_msg.wrist_position = Point(
            x=self.right_wrist_coords[0], 
            y=self.right_wrist_coords[1], 
            z=self.right_wrist_coords[2]
        )
        pose_msg.pose_detected = pose_detected
        pose_msg.hand_detected = hand_detected
        pose_msg.tracked_hand = "Right"
        pose_msg.shoulder_confidence = 0.8
        pose_msg.wrist_confidence = 0.9
        
        self.pose_coord_pub.publish(pose_msg)
        
        # Publish status
        status_msg = PoseTrackingStatus()
        status_msg.header = pose_msg.header
        status_msg.tracking_active = True
        status_msg.pose_detected = pose_detected
        status_msg.hand_detected = hand_detected
        status_msg.frame_count = self.frame_count
        status_msg.robot_enabled = self.running_state['robot_enabled']
        status_msg.status_message = "Active tracking with visual dashboard"
        
        elapsed_time = time.time() - self.start_time
        status_msg.fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0.0
        
        self.tracking_status_pub.publish(status_msg)
    
    def run_demo(self):
        """Main demo loop with visual interface"""
        print("🎬 Starting Complete CR3 Pose Recognition Demo")
        print("📱 Visual Dashboard + ROS2 Integration")
        print("=" * 60)
        print("🎯 Controls:")
        print("   - Click PAUSE/RESUME to pause tracking")
        print("   - Click STOP to exit")
        print("   - Click MIRROR to flip camera view")
        print("   - Press 'q' to quit")
        print("=" * 60)
        
        if not self.cap.isOpened():
            self.get_logger().error("❌ Cannot access camera!")
            return
        
        # Setup mouse callback
        ui_layout = None
        
        while self.running_state['running']:
            frame, shoulder_coords, wrist_coords = self.process_frame()
            if frame is None:
                break
            
            # Create UI with dashboard elements
            ui_frame, ui_layout = create_ui_elements(
                frame, 
                self.running_state, 
                shoulder_coords, 
                wrist_coords,
                robot_client=None  # We don't have a robot client in this demo
            )
            
            # Add performance info
            elapsed_time = time.time() - self.start_time
            fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
            pose_rate = self.pose_detections / self.frame_count if self.frame_count > 0 else 0
            hand_rate = self.hand_detections / self.frame_count if self.frame_count > 0 else 0
            
            # Add ROS2 status to UI panel
            frame_w = ui_layout['frame_width']
            cv2.putText(ui_frame, "ROS2 Integration:", (frame_w + 10, 320), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(ui_frame, f"Node: ACTIVE", (frame_w + 20, 350), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(ui_frame, f"Publishers: 2", (frame_w + 20, 380), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(ui_frame, f"Topics active", (frame_w + 20, 410), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # Performance metrics
            cv2.putText(ui_frame, "Performance:", (frame_w + 10, 450), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(ui_frame, f"FPS: {fps:.1f}", (frame_w + 20, 480), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(ui_frame, f"Pose: {pose_rate:.2f}", (frame_w + 20, 510), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(ui_frame, f"Hand: {hand_rate:.2f}", (frame_w + 20, 540), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            
            # Set up mouse callback
            if ui_layout:
                mouse_callback = create_mouse_callback(ui_layout)
                cv2.setMouseCallback('CR3 Pose Recognition Demo', mouse_callback, self.running_state)
            
            # Display
            cv2.imshow('CR3 Pose Recognition Demo', ui_frame)
            
            # Check for quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.running_state['running'] = False
        
        # Cleanup
        self.cap.release()
        cv2.destroyAllWindows()
        
        # Final report
        elapsed_time = time.time() - self.start_time
        final_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        print("\n" + "=" * 60)
        print("📊 DEMO COMPLETION REPORT:")
        print(f"   🎬 Total runtime: {elapsed_time:.1f} seconds")
        print(f"   📸 Frames processed: {self.frame_count}")
        print(f"   ⚡ Average FPS: {final_fps:.1f}")
        print(f"   🕺 Pose detections: {self.pose_detections} ({self.pose_detections/self.frame_count:.2f} rate)")
        print(f"   👋 Hand detections: {self.hand_detections} ({self.hand_detections/self.frame_count:.2f} rate)")
        print(f"   🤖 ROS2 messages published: {self.pose_detections * 2}")  # 2 messages per detection
        print("✅ Demo completed successfully!")

def main():
    rclpy.init()
    
    try:
        demo = CompletePoseRecognitionDemo()
        
        # Run demo in main thread
        demo.run_demo()
        
    except KeyboardInterrupt:
        print("\n⚠️  Demo interrupted by user")
    except Exception as e:
        print(f"\n💥 Error during demo: {e}")
    finally:
        if 'demo' in locals():
            demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
