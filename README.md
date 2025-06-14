# CR3_OpenCV_Controls

**Advanced robotic arm control system with real-time hand tracking using OpenCV and MediaPipe computer vision for DoBot CR3 robot manipulation in 3D space.**

**Version:** 4.0.0  
**Status:** 🟢 Production Ready - Phase 4 Complete  
**Architecture:** ROS2 Jazzy with Pose Recognition  
**Last Updated:** June 11, 2025

---

## 🎯 Project Overview

This system provides seamless control of a DoBot CR3 robotic arm through advanced pose recognition technology. The project features a modern ROS2 Jazzy Jalisco architecture with MediaPipe-based shoulder and wrist tracking for precise robot control.

### ✨ Key Features
- 🦴 **Advanced Pose Recognition** - MediaPipe pose + hand detection for shoulder-wrist tracking
- 🤖 **ROS2 Jazzy Integration** - Modern robotics middleware with custom messages
- 🔄 **Real-time Processing** - 30+ FPS pose tracking with coordinate transformation
- 🛡️ **Advanced Safety Systems** - Comprehensive workspace monitoring and alerts
- 🎮 **Debug Visualization** - Real-time pose overlay and performance monitoring
- 📊 **Performance Analytics** - Comprehensive system health and metrics

---

## 📁 Project Structure

```
robotic_arm_workspace/
├── README.md                          # This file
├── ros2_package/                     # ROS2 Jazzy package
│   ├── scripts/                      # Python nodes and scripts
│   ├── src/                          # C++ source files
│   ├── msg/                          # ROS2 message definitions
│   ├── srv/                          # ROS2 service definitions
│   ├── launch/                       # Launch files
│   ├── config/                       # Configuration files
│   ├── build/                        # ROS2 build artifacts
│   ├── install/                      # ROS2 installation files
│   ├── log/                          # Build and runtime logs
│   └── test/                         # Test files
├── Documentation/                     # Complete project documentation
│   ├── README.md                     # Documentation index
│   ├── Phases/                       # 10-phase implementation plan
│   ├── History/                      # Development timeline
│   └── References/                   # Technical references (5 files max)
├── Dashboards/                       # User interface applications
│   └── hand_tracking/               # OpenCV hand tracking dashboard
├── Robot_Controls/                   # Core robot control logic
│   ├── cr3_simple_controller.py     # Phase 1 implementation
│   └── motion_planning/             # Advanced motion control
├── Testing/                          # Comprehensive test suite
│   ├── unit/                        # Unit tests
│   ├── integration/                 # Integration tests
│   ├── safety/                      # Safety validation
│   └── utilities/                   # Test utilities
├── Config/                           # Configuration files
├── TCP-IP-ROS-6AXis/                # External ROS repository
└── Archive/                          # Historical reference
    ├── legacy_cr3_controls/         # Previous implementation
    └── legacy_hand_tracking/        # Legacy hand tracking code
```

---

## 🚀 Quick Start

### Prerequisites
- **ROS2 Jazzy Jalisco** (Ubuntu 24.04 LTS recommended)
- **Python 3.11+** with OpenCV, MediaPipe
- **DoBot CR3 Robot** with network connectivity
- **USB Camera** for hand tracking

### 1. Setup ROS2 Environment
```bash
# Source ROS2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# Build the ROS2 package
cd ros2_package
colcon build --packages-select cr3_hand_control
source install/setup.bash
```

### 2. Test Pose Recognition (Phase 4)
```bash
# Quick start pose recognition system
cd ros2_package/
./quick_start_pose_recognition.sh

# Or manually test pose recognition
source install/setup.bash
ros2 launch cr3_hand_control pose_recognition_test.launch.py
```

### 3. Run Legacy Hand Tracking Dashboard
```bash
# Start the original hand tracking interface (preserved)
cd Dashboards/hand_tracking/
python Hand_Tracking.py
```

### 4. Launch Complete System (Production)
```bash
# Launch full ROS2 system with pose recognition
source ros2_package/install/setup.bash
ros2 launch cr3_hand_control cr3_complete_system.launch.py
```
cd ros2_package
ros2 launch cr3_hand_control complete_system.launch.py
```

---

## 📚 Documentation

### Quick Navigation
- **[Getting Started Guide](Documentation/References/5_Startup_Usage_Reference.md)** - Setup and usage instructions
- **[System Architecture](Documentation/References/4_System_Architecture_Summary.md)** - Technical overview
- **[Implementation Plan](Documentation/10_phase_implementation_plan.md)** - 10-phase development roadmap
- **[File Organization](Documentation/References/3_File_Organization_Guidelines_Enhanced.md)** - Project structure guidelines

### Implementation Status
- ✅ **Phase 1**: Basic CR3 TCP connection
- 🔄 **Phase 2**: ROS2 infrastructure (in progress)
- ⏳ **Phases 3-10**: Planned development

---

## 🛠️ Development

### Current Implementation
- **ROS2 Package**: `cr3_hand_control` with custom messages
- **Hand Tracking**: OpenCV + MediaPipe implementation
- **Robot Control**: TCP-based communication with CR3
- **Safety Systems**: Basic collision detection

### Next Steps
1. Complete ROS2 parameter management
2. Implement advanced safety systems
3. Integrate hand tracking with robot control
4. Add performance optimization
5. Create comprehensive UI

---

## 🤝 Contributing

This project follows a structured 10-phase implementation plan. See `Documentation/Phases/` for detailed phase requirements.

### Development Guidelines
- Follow the file organization guidelines in `Documentation/References/`
- All new features should include corresponding tests in `Testing/`
- Update documentation for any structural changes
- Use ROS2 best practices for all robot communication

---

## 📄 License

MIT License - See [LICENSE](TCP-IP-ROS-6AXis/LICENSE) for details.

---

## 📞 Support

For technical questions and implementation guidance, refer to:
- **Documentation/References/** - Technical references
- **Documentation/History/** - Development timeline and decisions
- **Archive/** - Historical implementations and lessons learned

**Project Maintainer**: andrewlh  
**Email**: andrewlloydholland@gmail.com  
**Last Reorganization**: June 10, 2025
