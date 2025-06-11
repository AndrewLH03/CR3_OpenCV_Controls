# Startup and Usage Reference

## System Startup Guide

### Quick Start:
```bash
# Basic startup with robot test
python startup.py --robot-ip 192.168.1.6

# Skip robot test (faster startup)
python startup.py --robot-ip 192.168.1.6 --skip-robot-test

# Simulation mode (no robot required)
python startup.py --simulation

# Test system only
python startup.py --test-only

# Check dependencies only  
python startup.py --check-only
```

### Advanced Options:
```bash
# Custom hand tracking
python startup.py --robot-ip 192.168.1.6 --hand Left --mirror

# Custom port
python startup.py --robot-ip 192.168.1.6 --port 9999

# Show usage guide
python startup.py --usage
```

## System Operation Modes

### Production Mode:
- Connects to actual DoBot CR3 robot
- Performs robot movement test
- Initializes Phase 5 safety systems
- Real robot control with full safety

### Simulation Mode:
- No robot connection required
- Simulated robot movements
- Perfect for development and testing
- Full hand tracking interface

## Startup Simplification Complete

### Achievements:
- ✅ Single startup.py script for entire system
- ✅ Automatic dependency checking
- ✅ Comprehensive error handling
- ✅ Multiple operation modes
- ✅ Professional user guidance
- ✅ Safety system integration
- ✅ Testing framework integration

### User Experience:
- One-command system startup
- Clear status reporting
- Helpful error messages
- Multiple operation modes
- Professional system monitoring
