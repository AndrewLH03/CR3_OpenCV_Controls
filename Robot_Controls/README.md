# Robot Controls

This directory contains the core robot control implementation files for the CR3 robotic arm system.

## Current Implementation

### `cr3_simple_controller.py`
- **Purpose**: Phase 1 implementation of basic CR3 TCP connection and control
- **Features**: Direct TCP communication, basic movement commands, error handling
- **Status**: ✅ Functional - Basic robot control implemented

### `motion_planning/` (Directory)
- **Purpose**: Advanced motion planning and safety systems (Phase 5 implementation)
- **Status**: 📋 Planned - Directory structure ready for development

## Planned Implementation

The Robot_Controls directory will eventually contain:

### Core API Files
- `core_api.py` - TCP API + Connection Management
- `robot_controller.py` - Main robot control + Hand tracking integration  
- `utilities.py` - Helper functions and utilities
- `ros_bridge.py` - Complete ROS2 integration
- `migration_logger.py` - Logging system
- `__init__.py` - Module initialization

### Motion Planning Subdirectory
- `trajectory_planner.py` - Path planning algorithms
- `safety_systems.py` - Collision detection and emergency stops
- `coordinate_transformer.py` - Camera-to-robot coordinate mapping
- `README.md` - Motion planning documentation

## Usage

### Basic Robot Control (Phase 1)
```bash
cd Robot_Controls/
python cr3_simple_controller.py
```

### Integration with Hand Tracking
```bash
# Run hand tracking in one terminal
cd ../Dashboards/hand_tracking/
python Hand_Tracking.py

# Run robot control in another terminal  
cd ../Robot_Controls/
python cr3_simple_controller.py
```

## Development Guidelines

1. **Follow ROS2 conventions** for any new robot control implementations
2. **Add corresponding tests** in `../Testing/unit/` for new modules
3. **Update documentation** when adding new control features
4. **Implement safety checks** in all robot movement functions
5. **Use proper logging** via the migration_logger system

## Safety Notes

⚠️ **IMPORTANT**: All robot control files must implement proper safety measures:
- Emergency stop functionality
- Workspace boundary checks  
- Collision detection
- Connection timeout handling
- Error state recovery

## Next Development Phase

See `../Documentation/Phases/phase_02_ros2_infrastructure.md` for the next implementation steps.
