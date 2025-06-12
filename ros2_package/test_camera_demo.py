#!/usr/bin/env python3
"""
Camera Test Demo
Tests camera connectivity and shows MediaPipe pose/hand detection
"""

import cv2
import mediapipe as mp
import numpy as np
import time

def test_camera_and_pose():
    """Test camera connectivity and MediaPipe functionality"""
    print("🎥 Testing Camera and Pose Recognition...")
    print("=" * 50)
    
    # Initialize MediaPipe
    print("📦 Initializing MediaPipe...")
    mp_hands = mp.solutions.hands
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    
    hands = mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    
    pose = mp_pose.Pose(
        static_image_mode=False,
        model_complexity=1,
        smooth_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    
    # Test camera connectivity
    print("🎯 Testing camera connectivity...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ ERROR: Cannot access camera!")
        print("   - Check if camera is connected")
        print("   - Check if another application is using the camera")
        return False
    
    # Get camera info
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"✅ Camera found: {width}x{height} @ {fps}fps")
    print(f"🔍 Testing pose and hand detection...")
    print(f"💡 Position yourself in view of the camera")
    print(f"🚀 Press 'q' to quit, 'SPACE' to take screenshot")
    print("=" * 50)
    
    frame_count = 0
    pose_detections = 0
    hand_detections = 0
    start_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Failed to capture frame")
            break
        
        frame_count += 1
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process with MediaPipe
        pose_results = pose.process(rgb_frame)
        hand_results = hands.process(rgb_frame)
        
        # Create display frame
        display_frame = frame.copy()
        
        # Track detections
        pose_detected = False
        hand_detected = False
        
        # Draw pose landmarks
        if pose_results.pose_landmarks:
            pose_detected = True
            pose_detections += 1
            
            mp_drawing.draw_landmarks(
                display_frame,
                pose_results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(255, 255, 0), thickness=4, circle_radius=6),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(255, 255, 255), thickness=2)
            )
            
            # Highlight shoulder and wrist
            landmarks = pose_results.pose_landmarks.landmark
            shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
            
            h, w = display_frame.shape[:2]
            shoulder_px = (int(shoulder.x * w), int(shoulder.y * h))
            wrist_px = (int(wrist.x * w), int(wrist.y * h))
            
            cv2.circle(display_frame, shoulder_px, 12, (0, 0, 255), -1)
            cv2.circle(display_frame, wrist_px, 12, (255, 0, 0), -1)
            cv2.putText(display_frame, "SHOULDER", (shoulder_px[0]+15, shoulder_px[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(display_frame, "WRIST", (wrist_px[0]+15, wrist_px[1]), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
            cv2.line(display_frame, shoulder_px, wrist_px, (255, 0, 255), 4)
        
        # Draw hand landmarks
        if hand_results.multi_hand_landmarks:
            hand_detected = True
            hand_detections += 1
            
            for hand_landmarks in hand_results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    display_frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=5, circle_radius=8),
                    connection_drawing_spec=mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=3)
                )
        
        # Calculate performance
        elapsed_time = time.time() - start_time
        current_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        pose_rate = pose_detections / frame_count if frame_count > 0 else 0
        hand_rate = hand_detections / frame_count if frame_count > 0 else 0
        
        # Add status overlay
        status_y = 30
        cv2.putText(display_frame, f"FPS: {current_fps:.1f}", (10, status_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        
        status_y += 40
        pose_color = (0, 255, 0) if pose_detected else (0, 0, 255)
        cv2.putText(display_frame, f"POSE: {'DETECTED' if pose_detected else 'NOT FOUND'}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, pose_color, 2)
        
        status_y += 40
        hand_color = (0, 255, 0) if hand_detected else (0, 0, 255)
        cv2.putText(display_frame, f"HAND: {'DETECTED' if hand_detected else 'NOT FOUND'}", 
                   (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, hand_color, 2)
        
        status_y += 40
        cv2.putText(display_frame, f"Pose Rate: {pose_rate:.2f}", (10, status_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        status_y += 30
        cv2.putText(display_frame, f"Hand Rate: {hand_rate:.2f}", (10, status_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Instructions
        instructions_y = display_frame.shape[0] - 60
        cv2.putText(display_frame, "Press 'q' to quit, SPACE for screenshot", 
                   (10, instructions_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Show frame
        cv2.imshow('CR3 Pose Recognition Test', display_frame)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' '):
            # Save screenshot
            timestamp = int(time.time())
            filename = f"pose_detection_screenshot_{timestamp}.jpg"
            cv2.imwrite(filename, display_frame)
            print(f"📸 Screenshot saved: {filename}")
    
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    
    # Final report
    print("\n" + "=" * 50)
    print("📊 FINAL RESULTS:")
    print(f"   Total frames: {frame_count}")
    print(f"   Average FPS: {current_fps:.1f}")
    print(f"   Pose detection rate: {pose_rate:.2f} ({pose_detections}/{frame_count})")
    print(f"   Hand detection rate: {hand_rate:.2f} ({hand_detections}/{frame_count})")
    print("✅ Camera and pose recognition test completed!")
    return True

if __name__ == "__main__":
    try:
        success = test_camera_and_pose()
        if success:
            print("\n🎉 All systems working correctly!")
        else:
            print("\n❌ System test failed!")
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
    except Exception as e:
        print(f"\n💥 Error during test: {e}")
