#!/usr/bin/env python3
"""
Text-based Pose Recognition Monitor
Demonstrates the pose recognition system working without GUI requirements
"""

import cv2
import mediapipe as mp
import time
import sys

def monitor_pose_recognition():
    """Monitor pose recognition and print status updates"""
    print("🎥 CR3 Pose Recognition System Monitor")
    print("=" * 60)
    print("📦 Initializing MediaPipe...")
    
    # Initialize MediaPipe
    mp_hands = mp.solutions.hands
    mp_pose = mp.solutions.pose
    
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
    
    print("✅ MediaPipe models loaded successfully!")
    
    # Test camera
    print("🎯 Testing camera connectivity...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("❌ ERROR: Cannot access camera!")
        print("   Possible issues:")
        print("   - Camera not connected")
        print("   - Camera in use by another application")
        print("   - Permissions issue")
        return False
    
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"✅ Camera connected: {width}x{height} @ {fps}fps")
    print("🚀 Starting pose recognition monitoring...")
    print("   Press Ctrl+C to stop")
    print("=" * 60)
    
    # Monitoring variables
    frame_count = 0
    pose_detections = 0
    hand_detections = 0
    start_time = time.time()
    last_report_time = start_time
    report_interval = 5.0  # Report every 5 seconds
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("⚠️  Warning: Failed to capture frame")
                continue
            
            frame_count += 1
            
            # Convert to RGB for MediaPipe
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process with MediaPipe
            pose_results = pose.process(rgb_frame)
            hand_results = hands.process(rgb_frame)
            
            # Track detections
            pose_detected = False
            hand_detected = False
            shoulder_coords = None
            wrist_coords = None
            
            # Check pose detection
            if pose_results.pose_landmarks:
                pose_detected = True
                pose_detections += 1
                
                # Get shoulder and wrist coordinates
                landmarks = pose_results.pose_landmarks.landmark
                shoulder = landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER]
                wrist = landmarks[mp_pose.PoseLandmark.RIGHT_WRIST]
                
                shoulder_coords = (shoulder.x, shoulder.y, shoulder.z)
                wrist_coords = (wrist.x, wrist.y, wrist.z)
            
            # Check hand detection
            if hand_results.multi_hand_landmarks and hand_results.multi_handedness:
                for hand_landmarks, hand_info in zip(hand_results.multi_hand_landmarks, hand_results.multi_handedness):
                    label = hand_info.classification[0].label
                    label = "Left" if label == "Right" else "Right"  # MediaPipe flips labels
                    
                    if label == "Right":
                        hand_detected = True
                        hand_detections += 1
                        
                        # Get more precise wrist coordinates from hand
                        wrist_landmark = hand_landmarks.landmark[mp_hands.HandLandmark.WRIST]
                        wrist_coords = (wrist_landmark.x, wrist_landmark.y, wrist_landmark.z)
                        break
            
            # Generate periodic reports
            current_time = time.time()
            if current_time - last_report_time >= report_interval:
                elapsed_time = current_time - start_time
                fps = frame_count / elapsed_time
                pose_rate = pose_detections / frame_count if frame_count > 0 else 0
                hand_rate = hand_detections / frame_count if frame_count > 0 else 0
                
                print(f"📊 Status Report (t={elapsed_time:.1f}s):")
                print(f"   FPS: {fps:.1f} | Frames: {frame_count}")
                print(f"   Pose Detection Rate: {pose_rate:.2f} ({pose_detections}/{frame_count})")
                print(f"   Hand Detection Rate: {hand_rate:.2f} ({hand_detections}/{frame_count})")
                
                if pose_detected:
                    print(f"   ✅ Current Detection: POSE + {'HAND' if hand_detected else 'NO HAND'}")
                    if shoulder_coords and wrist_coords:
                        # Convert to robot coordinates (simplified)
                        robot_shoulder = (
                            200 + (shoulder_coords[0] - 0.5) * 400,  # X in mm
                            0 + (0.5 - shoulder_coords[1]) * 300,    # Y in mm  
                            150 + shoulder_coords[2] * 200           # Z in mm
                        )
                        robot_wrist = (
                            200 + (wrist_coords[0] - 0.5) * 400,     # X in mm
                            0 + (0.5 - wrist_coords[1]) * 300,       # Y in mm
                            150 + wrist_coords[2] * 200              # Z in mm
                        )
                        print(f"   🤖 Robot Coords - Shoulder: ({robot_shoulder[0]:.0f}, {robot_shoulder[1]:.0f}, {robot_shoulder[2]:.0f})mm")
                        print(f"   🤖 Robot Coords - Wrist: ({robot_wrist[0]:.0f}, {robot_wrist[1]:.0f}, {robot_wrist[2]:.0f})mm")
                else:
                    print(f"   ❌ Current Detection: NO POSE DETECTED")
                
                print("   " + "-" * 50)
                last_report_time = current_time
            
            # Show real-time status for active detections
            if pose_detected or hand_detected:
                status_chars = []
                if pose_detected:
                    status_chars.append("🕺")
                if hand_detected:
                    status_chars.append("👋")
                print(f"\r{''.join(status_chars)} Frame {frame_count} - Active detection!", end="", flush=True)
            else:
                print(f"\r⏳ Frame {frame_count} - Searching for person...", end="", flush=True)
            
            # Small delay to prevent overwhelming output
            time.sleep(0.033)  # ~30fps monitoring
    
    except KeyboardInterrupt:
        print(f"\n\n⚠️  Monitoring stopped by user")
    
    finally:
        # Final report
        elapsed_time = time.time() - start_time
        final_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
        
        print("\n" + "=" * 60)
        print("📋 FINAL MONITORING REPORT:")
        print(f"   ⏱️  Total runtime: {elapsed_time:.1f} seconds")
        print(f"   📸 Total frames: {frame_count}")
        print(f"   ⚡ Average FPS: {final_fps:.1f}")
        print(f"   🕺 Pose detections: {pose_detections} ({pose_detections/frame_count:.2f} rate)")
        print(f"   👋 Hand detections: {hand_detections} ({hand_detections/frame_count:.2f} rate)")
        
        if pose_detections > 0:
            print(f"   ✅ System working correctly - pose tracking functional!")
        else:
            print(f"   ⚠️  No poses detected - check if person is visible to camera")
        
        print("=" * 60)
        
        cap.release()
        return pose_detections > 0

if __name__ == "__main__":
    print("🎬 Starting CR3 Pose Recognition Monitor")
    print("📱 This demonstrates the core tracking functionality")
    print()
    
    success = monitor_pose_recognition()
    
    if success:
        print("🎉 Pose recognition system verification PASSED!")
        print("✅ The system is ready for full dashboard and ROS2 integration!")
    else:
        print("⚠️  Pose recognition system needs attention")
        print("💡 Try positioning yourself clearly in view of the camera")
    
    print("\n🔗 Next steps:")
    print("   1. Full visual dashboard available in complete_demo.py")
    print("   2. ROS2 integration available in pose_recognition_node.py") 
    print("   3. Launch system available in launch/pose_recognition.launch.py")
