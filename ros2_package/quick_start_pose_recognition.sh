#!/bin/bash
# Quick Start Script for CR3 Pose Recognition System
# Run this after Phase 4 completion to test pose recognition

echo "============================================================"
echo "CR3 Pose Recognition System - Quick Start Test"
echo "============================================================"

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo "❌ Please run this script from the ros2_package directory"
    exit 1
fi

echo "🔧 Building ROS2 package..."
colcon build --packages-select cr3_hand_control

if [ $? -ne 0 ]; then
    echo "❌ Build failed. Please check for errors above."
    exit 1
fi

echo "✅ Build successful!"
echo ""

echo "🔧 Sourcing workspace..."
source install/setup.bash

echo "🧪 Running ROS2 integration tests..."
python3 scripts/test_ros2_integration.py

echo ""
echo "📋 Available executables:"
ros2 pkg executables cr3_hand_control

echo ""
echo "📨 Available messages:"
echo "  - cr3_hand_control/msg/PoseCoordinates"
echo "  - cr3_hand_control/msg/PoseTrackingStatus" 
echo "  - cr3_hand_control/msg/DebugInfo"

echo ""
echo "🚀 To test pose recognition (requires MediaPipe and camera):"
echo "  1. Install MediaPipe: pip install mediapipe (in virtual env)"
echo "  2. Connect USB camera"
echo "  3. Run: ros2 launch cr3_hand_control pose_recognition_test.launch.py"

echo ""
echo "📖 For more information, see:"
echo "  - Documentation/References/PHASE4_COMPLETION_REPORT.md"
echo "  - Documentation/References/Phase4_Pose_Recognition_Implementation.md"
echo ""
echo "🎉 Phase 4 pose recognition system ready!"
echo "============================================================"
