#!/bin/bash
# Install dependencies for pose recognition
# Based on working Dashboards implementation requirements

echo "Installing Pose Recognition Dependencies..."

# Check if running in a virtual environment
if [[ "$VIRTUAL_ENV" != "" ]]; then
    echo "Using virtual environment: $VIRTUAL_ENV"
    PIP_CMD="pip"
else
    echo "Using system Python - you may need sudo rights"
    PIP_CMD="pip3"
fi

# Core computer vision dependencies
echo "Installing OpenCV..."
$PIP_CMD install opencv-python>=4.5.0

echo "Installing MediaPipe..."
$PIP_CMD install mediapipe>=0.10.0

echo "Installing NumPy..."
$PIP_CMD install numpy>=1.21.0

# Additional dependencies for better performance
echo "Installing additional dependencies..."
$PIP_CMD install pillow>=8.0.0

# Verify installations
echo "Verifying installations..."

python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')" || echo "ERROR: OpenCV not installed properly"
python3 -c "import mediapipe as mp; print(f'MediaPipe version: {mp.__version__}')" || echo "ERROR: MediaPipe not installed properly"
python3 -c "import numpy as np; print(f'NumPy version: {np.__version__}')" || echo "ERROR: NumPy not installed properly"

echo "Pose recognition dependencies installation complete!"
echo ""
echo "To test the installation, you can run:"
echo "  ros2 launch cr3_hand_control pose_recognition_test.launch.py"
