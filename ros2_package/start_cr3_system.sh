#!/bin/bash

# CR3 Hand Tracking Control System Startup Script
# This script sets up the environment, validates the system, and starts the CR3 control system

set -e

echo "🚀 CR3 Hand Tracking Control System Startup"
echo "=============================================="

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
ROS2_PACKAGE_DIR="$SCRIPT_DIR"

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}Step 1: Setting up ROS2 environment...${NC}"

# Source ROS2 installation
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ ROS2 Jazzy sourced${NC}"
else
    echo -e "${RED}✗ ROS2 Jazzy not found!${NC}"
    exit 1
fi

# Source local package installation
LOCAL_SETUP="$ROS2_PACKAGE_DIR/install/setup.bash"
if [ -f "$LOCAL_SETUP" ]; then
    echo "Sourcing local package installation: $LOCAL_SETUP"
    source "$LOCAL_SETUP"
    echo -e "${GREEN}✓ Local package overlay sourced${NC}"
else
    echo -e "${RED}ERROR: Local package not installed!${NC}"
    echo -e "${RED}Missing: $LOCAL_SETUP${NC}"
    echo -e "${YELLOW}Please run 'colcon build' first${NC}"
    exit 1
fi

echo -e "${BLUE}Step 2: Validating system components...${NC}"

# Check if the package is available
echo "Checking if cr3_hand_control package is available..."
if python3 -c "from cr3_hand_control.msg import RobotStatus; print('✓ Package import successful')" 2>/dev/null; then
    echo -e "${GREEN}✓ cr3_hand_control package is properly installed${NC}"
else
    echo -e "${RED}✗ cr3_hand_control package import failed${NC}"
    echo "ROS_PACKAGE_PATH: $ROS_PACKAGE_PATH"
    echo "AMENT_PREFIX_PATH: $AMENT_PREFIX_PATH"
    exit 1
fi

# Check for required executables
echo "Checking C++ executables..."
EXECUTABLES=("basic_robot_controller" "emergency_stop_handler" "workspace_validator" "coordinate_broadcaster")
for exe in "${EXECUTABLES[@]}"; do
    if [ -f "$ROS2_PACKAGE_DIR/install/cr3_hand_control/lib/cr3_hand_control/$exe" ]; then
        echo -e "${GREEN}✓ $exe executable found${NC}"
    else
        echo -e "${YELLOW}⚠ $exe executable not found (may need rebuild)${NC}"
    fi
done

# Parse command line arguments
INTEGRATION_TEST=false
START_SAFETY=false
START_FULL_SYSTEM=false
HELP=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -t|--test)
            INTEGRATION_TEST=true
            shift
            ;;
        -s|--safety)
            START_SAFETY=true
            shift
            ;;
        -f|--full)
            START_FULL_SYSTEM=true
            shift
            ;;
        -h|--help)
            HELP=true
            shift
            ;;
        *)
            echo "Unknown option $1"
            HELP=true
            shift
            ;;
    esac
done

if [ "$HELP" = true ]; then
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -t, --test     Run integration test only"
    echo "  -s, --safety   Start safety system only"
    echo "  -f, --full     Start complete Phase 3 system"
    echo "  -h, --help     Show this help message"
    echo ""
    echo "Default (no options): Run integration test"
    exit 0
fi

# Default to integration test if no options specified
if [ "$INTEGRATION_TEST" = false ] && [ "$START_SAFETY" = false ] && [ "$START_FULL_SYSTEM" = false ]; then
    INTEGRATION_TEST=true
fi

if [ "$INTEGRATION_TEST" = true ]; then
    echo -e "${BLUE}Step 3: Running integration test...${NC}"
    cd "$ROS2_PACKAGE_DIR"
    python3 test/integration/test_phase3_integration.py
    echo -e "${GREEN}✓ Integration test completed successfully!${NC}"
fi

if [ "$START_SAFETY" = true ]; then
    echo -e "${BLUE}Step 3: Starting safety system...${NC}"
    echo -e "${YELLOW}Starting safety monitor (Ctrl+C to stop)${NC}"
    cd "$ROS2_PACKAGE_DIR"
    python3 scripts/safety/safety_monitor.py
fi

if [ "$START_FULL_SYSTEM" = true ]; then
    echo -e "${BLUE}Step 3: Starting complete Phase 3 system...${NC}"
    echo -e "${YELLOW}Launching Phase 3 complete system (Ctrl+C to stop)${NC}"
    cd "$ROS2_PACKAGE_DIR"
    ros2 launch cr3_hand_control complete_system.launch.py
fi

echo -e "${GREEN}🎯 CR3 system ready!${NC}"
