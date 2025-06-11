#!/bin/bash

# Phase 3 Integration Test Runner
# This script properly sources ROS2 and runs the integration test

set -e

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROS2_PACKAGE_DIR="$SCRIPT_DIR"

# Source ROS2 installation
source /opt/ros/jazzy/setup.bash

# Source local package installation
LOCAL_SETUP="$ROS2_PACKAGE_DIR/install/setup.bash"
if [ -f "$LOCAL_SETUP" ]; then
    echo "Sourcing local package installation: $LOCAL_SETUP"
    source "$LOCAL_SETUP"
else
    echo "ERROR: Local package not installed!"
    echo "Missing: $LOCAL_SETUP"
    echo "Please run 'colcon build' first"
    exit 1
fi

# Check if the package is available
echo "Checking if cr3_hand_control package is available..."
if python3 -c "from cr3_hand_control.msg import RobotStatus; print('✓ Package import successful')" 2>/dev/null; then
    echo "✓ cr3_hand_control package is properly installed"
else
    echo "✗ cr3_hand_control package import failed"
    echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
    echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
    exit 1
fi

# Run the integration test
echo "Running Phase 3 Integration Test..."
cd "$ROS2_PACKAGE_DIR"
python3 test/test_phase3_integration.py

echo "Integration test completed!"
