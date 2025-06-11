# Testing Directory

Comprehensive test suite for the CR3 robotic arm hand tracking control system.

## Directory Structure

### `unit/` - Unit Tests
- **Purpose**: Test individual components and functions in isolation
- **Naming**: `test_{component_name}.py`
- **Examples**: `test_robot_controller.py`, `test_core_api.py`, `test_utilities.py`

### `integration/` - Integration Tests  
- **Purpose**: Test system components working together
- **Naming**: `{system}_integration_tests.py`
- **Examples**: `robot_integration_tests.py`, `hand_tracking_integration.py`

### `safety/` - Safety Validation Tests
- **Purpose**: Validate safety systems and emergency procedures
- **Examples**: `safety_tests.py`, `emergency_stop_tests.py`, `collision_detection_tests.py`

### `utilities/` - Test Utilities
- **Purpose**: Shared testing utilities and helper functions
- **Examples**: `test_utils.py`, `robot_testing_utils.py`, `mock_robot.py`

## Running Tests

### All Tests
```bash
# From project root
python -m pytest Testing/ -v
```

### Specific Test Categories
```bash
# Unit tests only
python -m pytest Testing/unit/ -v

# Integration tests only  
python -m pytest Testing/integration/ -v

# Safety tests only
python -m pytest Testing/safety/ -v
```

### Individual Test Files
```bash
# Test specific component
python -m pytest Testing/unit/test_robot_controller.py -v
```

## Test Development Guidelines

### 1. Test Naming Convention
- **Files**: `test_{component}.py`
- **Classes**: `Test{ComponentName}`  
- **Methods**: `test_{specific_functionality}`

### 2. Test Structure
```python
import pytest
from unittest.mock import Mock, patch
from Robot_Controls.robot_controller import RobotController

class TestRobotController:
    def setup_method(self):
        """Setup for each test method"""
        self.robot = RobotController()
    
    def test_connection_establishment(self):
        """Test robot connection functionality"""
        # Arrange, Act, Assert pattern
        pass
    
    def teardown_method(self):
        """Cleanup after each test method"""
        pass
```

### 3. Safety Test Requirements
All safety tests must verify:
- ✅ Emergency stop functionality
- ✅ Workspace boundary enforcement
- ✅ Collision detection accuracy
- ✅ Connection timeout handling
- ✅ Error state recovery

### 4. Mock Usage for Hardware
```python
# Always mock hardware interactions in tests
@patch('Robot_Controls.core_api.socket.socket')
def test_robot_communication(self, mock_socket):
    # Test logic without actual robot
    pass
```

## Test Coverage Goals

- **Unit Tests**: 90%+ code coverage
- **Integration Tests**: All major workflows covered
- **Safety Tests**: 100% safety feature coverage
- **Performance Tests**: Response time validation

## Continuous Integration

Tests are automatically run on:
- Code commits
- Pull requests  
- Nightly builds
- Release preparations

## Adding New Tests

1. **Create test file** in appropriate subdirectory
2. **Follow naming conventions** and structure guidelines
3. **Include docstrings** explaining test purpose
4. **Add to CI pipeline** if needed
5. **Update this README** if adding new test categories

## Testing Tools Used

- **pytest** - Main testing framework
- **unittest.mock** - Mocking and patching
- **pytest-cov** - Coverage reporting
- **pytest-xdist** - Parallel test execution

## Safety Testing Notes

⚠️ **CRITICAL**: Never run safety tests with real robot hardware unless in a controlled environment with proper safety measures in place.
