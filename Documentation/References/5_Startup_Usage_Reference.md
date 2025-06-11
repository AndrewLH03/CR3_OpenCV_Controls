# Startup and Usage Reference - CR3 Hand Tracking Control System

## Current System Status: Phase 3 Complete

### ROS2 System Startup Guide

#### Universal Startup Script:
```bash
# Navigate to ROS2 package
cd /path/to/CR3_OpenCV_Controls/ros2_package

# Test mode - Run integration tests
./start_cr3_system.sh --test

# Safety system only
./start_cr3_system.sh --safety

# Complete system launch
./start_cr3_system.sh --full

# Help and options
./start_cr3_system.sh --help
```

#### Manual ROS2 Launch Commands:
```bash
# Source ROS2 environment
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch complete system
ros2 launch cr3_hand_control complete_system.launch.py

# Launch individual subsystems
ros2 launch cr3_hand_control safety_system.launch.py
ros2 launch cr3_hand_control coordinate_system.launch.py
ros2 launch cr3_hand_control robot_control_system.launch.py

# Run integration tests
python3 test/integration/test_phase3_integration.py
python startup.py --usage
```

#### Build and Setup:
```bash
# Build the ROS2 package
cd /path/to/CR3_OpenCV_Controls/ros2_package
colcon build --symlink-install

# Source the built package
source install/setup.bash
```

## System Operation Modes

### Test Mode (`--test`):
- Runs comprehensive integration tests
- Validates all safety systems
- Tests robot communication (if connected)
- Reports system health status

### Safety Mode (`--safety`):
- Launches safety monitoring only
- Workspace boundary validation
- Emergency stop systems
- Real-time safety alerts

### Full System Mode (`--full`):
- Complete system deployment
- All domains active (robot_control, safety, coordination)
- Ready for robot operation
- Comprehensive monitoring

## Current System Capabilities

### Functional Domains:
- **Robot Control**: TCP communication with CR3 robot
- **Safety Monitoring**: Real-time workspace validation
- **Coordinate Management**: Transform and frame handling
- **Testing Framework**: Comprehensive validation

### Validation Results:
```
✅ Integration Test Results:
├── Total alerts received: 5
├── Boundary alerts: 2  
├── Emergency alerts: 2
├── Service calls: ✅ Successful
└── All safety systems: ✅ Operational
```

## Phase 4 Preparation

### Ready for Hand Tracking:
- **Perception domain**: Structure prepared
- **Camera configuration**: Files ready
- **Launch system**: Perception subsystem prepared
- **Message definitions**: HandPosition.msg defined
- **Coordinate mapping**: Framework established

### Next Steps:
1. Implement camera node in perception domain
2. Add MediaPipe hand tracking integration  
3. Activate perception launch files
4. Integrate with coordinate transformation system
5. Validate hand-to-robot coordinate mapping

## Troubleshooting

### Common Issues:
1. **Build failures**: Ensure ROS2 Jazzy is sourced
2. **Node startup issues**: Check configuration file paths
3. **Robot connection**: Verify CR3 robot IP (192.168.1.6)
4. **Permission errors**: Ensure executable permissions on scripts

### System Requirements:
- ROS2 Jazzy Jalisco
- Ubuntu 22.04+ or compatible Linux distribution
- Network access to CR3 robot (if using real robot)
- Python 3.10+ for script components

### User Experience:
- One-command system startup
- Clear status reporting
- Helpful error messages
- Multiple operation modes
- Professional system monitoring
