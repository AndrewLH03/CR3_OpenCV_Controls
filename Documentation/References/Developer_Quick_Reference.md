# Developer Quick Reference - CR3 Hand Tracking Control System

## 🚀 Quick Start Commands

### Essential Commands:
```bash
# Navigate to package
cd /path/to/CR3_OpenCV_Controls/ros2_package

# Build package
colcon build --symlink-install

# Source environment
source install/setup.bash

# Test pose recognition (Phase 4)
./quick_start_pose_recognition.sh

# Run integration test
./start_cr3_system.sh --test

# Launch complete system
./start_cr3_system.sh --full
```

## 📁 File Organization Quick Reference

### Where to Find Things:
```
🤖 Robot Control Code:
├── C++: src/robot_control/
├── Python: scripts/robot_control/
├── Config: config/robot/
└── Tests: test/unit/robot_control/

🛡️ Safety Systems:
├── C++: src/safety/
├── Python: scripts/safety/
├── Config: config/safety/
└── Tests: test/unit/safety/

🎯 Coordinate Management:
├── C++: src/coordination/
├── Python: scripts/coordination/
├── Config: config/coordination/
└── Tests: test/unit/coordination/

👁️ Pose Recognition (Phase 4 ✅):
├── Node: src/cr3_hand_control/pose_recognition_node.py
├── Messages: msg/PoseCoordinates.msg, PoseTrackingStatus.msg
├── Config: config/perception/pose_recognition_params.yaml
├── Launch: launch/testing/pose_recognition_test.launch.py
└── Reference: Documentation/References/working_implementations/
```

### Launch Files:
```
🚀 System Launches:
└── launch/systems/complete_system.launch.py

🔧 Subsystem Launches:
├── launch/subsystems/robot_control_system.launch.py
├── launch/subsystems/safety_system.launch.py
├── launch/subsystems/coordinate_system.launch.py
└── launch/subsystems/perception_system.launch.py

🧪 Testing Launches:
└── launch/testing/test_nodes.launch.py
```

## 🔧 Development Workflow

### Adding New Functionality:

#### 1. Robot Control Features:
```bash
# Add C++ node
vim src/robot_control/new_feature.cpp

# Add Python script
vim scripts/robot_control/new_feature.py

# Update configuration
vim config/robot/feature_config.yaml

# Add to CMakeLists.txt for C++
vim CMakeLists.txt

# Test integration
./start_cr3_system.sh --test
```

#### 2. Safety Features:
```bash
# Add safety monitoring
vim src/safety/new_safety_feature.cpp

# Update safety config
vim config/safety/safety_params_ros2.yaml

# Add safety tests
vim test/unit/safety/test_new_feature.py
```

#### 3. Phase 4 - Hand Tracking:
```bash
# Camera integration
vim src/perception/camera_node.cpp

# Hand tracking script
vim scripts/perception/hand_tracking_node.py

# Activate perception launch
# Edit launch/subsystems/perception_system.launch.py
```

## 🧪 Testing Quick Reference

### Running Tests:
```bash
# Integration test (recommended)
./start_cr3_system.sh --test

# Manual test execution
python3 test/integration/test_phase3_integration.py

# Unit tests (when available)
python3 test/unit/domain/test_specific_feature.py

# Essential message tests
python3 test/shared/test_essential_messages.py
```

### Test Results Interpretation:
```
✅ Expected Results:
├── Total alerts received: 5
├── Boundary alerts: 2
├── Emergency alerts: 2  
├── Speed limit alerts: 1
├── Service calls successful: True
└── All safety systems operational
```

## 📝 Message and Service Reference

### Current Messages:
```
Robot Domain:
├── robot/BasicCommand.msg
└── robot/RobotStatus.msg

Safety Domain:
├── safety/SafetyAlert.msg
└── safety/SafetyStatus.msg

Perception Domain:
└── perception/HandPosition.msg

Coordination Domain:
└── coordination/CoordinateTransform.msg
```

### Current Services:
```
Robot Domain:
└── robot/SetParameters.srv

Safety Domain:
└── safety/EmergencyStop.srv
```

## 🔧 Configuration Reference

### Robot Configuration:
```yaml
# config/robot/cr3_parameters.yaml
robot_controller:
  ros__parameters:
    tcp_settings:
      dashboard_port: 29999
      move_port: 30003
      feedback_port: 30004
    joint_limits: [...]
    position_limits: [...]
```

### Safety Configuration:
```yaml
# config/safety/workspace_boundaries.yaml
workspace_validator:
  ros__parameters:
    workspace_limits:
      x_min: -0.6
      x_max: 0.6
      # ... more limits
```

## 🐛 Debugging Quick Reference

### Common Issues and Solutions:

#### Build Issues:
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install

# Source ROS2
source /opt/ros/jazzy/setup.bash
```

#### Runtime Issues:
```bash
# Check node status
ros2 node list

# Check topics
ros2 topic list

# Monitor messages
ros2 topic echo /safety_alerts

# Check services
ros2 service list
```

#### Configuration Issues:
```bash
# Validate config files
ros2 param list

# Check parameter values
ros2 param get /node_name parameter_name
```

## 🚀 Performance Tips

### Optimization:
- Use `--symlink-install` for faster development builds
- Run integration tests regularly to catch issues early
- Monitor system resource usage during development
- Use domain-specific launches for focused development

### Best Practices:
- Follow functional domain organization
- Add comprehensive error handling
- Include logging for debugging
- Validate configurations before deployment
- Use safety systems consistently

## 📚 Documentation Structure

### Key Documents:
```
Documentation/
├── References/
│   ├── 10_Phase_Implementation_Plan.md
│   ├── 4_System_Architecture_Summary.md
│   ├── 5_Startup_Usage_Reference.md
│   ├── Current_Project_Status.md
│   ├── FUNCTIONAL_REORGANIZATION_REPORT.md
│   └── Developer_Quick_Reference.md [This file]
├── Phases/ [Phase-specific documentation]
└── Explanations/ [Detailed system explanations]
```

## 🎯 Phase 4 Development Guide

### Ready for Implementation:
1. **Camera Integration**: Use existing perception domain structure
2. **MediaPipe Setup**: Add to scripts/perception/
3. **Coordinate Mapping**: Leverage existing coordination domain
4. **Launch Integration**: Activate perception_system.launch.py
5. **Testing**: Extend integration tests for hand tracking

### Implementation Checklist:
- [ ] Camera node implementation
- [ ] MediaPipe hand tracking integration  
- [ ] Hand position message publishing
- [ ] Coordinate transformation validation
- [ ] Integration with safety systems
- [ ] Performance optimization
- [ ] Comprehensive testing

---

**Quick Help**: For immediate assistance, run `./start_cr3_system.sh --help` or refer to the phase implementation plan for detailed guidance.
