#!/usr/bin/env python3
"""
Simple test for ROS2 pose recognition integration
Tests the package structure and message compilation without requiring MediaPipe
"""

import subprocess
import os
import sys

def test_ros2_package():
    """Test ROS2 package compilation and structure"""
    print("Testing ROS2 package structure...")
    
    try:
        # Test if the executable exists
        result = subprocess.run(
            ['ros2', 'pkg', 'executables', 'cr3_hand_control'], 
            capture_output=True, 
            text=True, 
            timeout=10
        )
        
        if 'pose_recognition_node' in result.stdout:
            print("✅ Pose recognition node executable found")
            return True
        else:
            print("❌ Pose recognition node executable not found")
            print(f"Available executables: {result.stdout}")
            return False
            
    except subprocess.TimeoutExpired:
        print("❌ ROS2 command timeout")
        return False
    except Exception as e:
        print(f"❌ ROS2 package test failed: {e}")
        return False

def test_launch_files():
    """Test launch file availability"""
    print("Testing launch files...")
    
    try:
        # Test if launch files exist
        result = subprocess.run(
            ['ros2', 'pkg', 'prefix', 'cr3_hand_control'], 
            capture_output=True, 
            text=True, 
            timeout=10
        )
        
        if result.returncode == 0:
            pkg_path = result.stdout.strip()
            launch_path = os.path.join(pkg_path, 'share', 'cr3_hand_control', 'launch')
            
            if os.path.exists(launch_path):
                launch_files = os.listdir(launch_path)
                pose_launches = [f for f in launch_files if 'pose' in f.lower()]
                
                if pose_launches:
                    print(f"✅ Pose recognition launch files found: {pose_launches}")
                    return True
                else:
                    print(f"❌ No pose recognition launch files found in {launch_path}")
                    return False
            else:
                print(f"❌ Launch directory not found: {launch_path}")
                return False
        else:
            print("❌ Could not find package path")
            return False
            
    except Exception as e:
        print(f"❌ Launch file test failed: {e}")
        return False

def test_messages():
    """Test custom message definitions"""
    print("Testing custom messages...")
    
    try:
        result = subprocess.run(
            ['ros2', 'interface', 'show', 'cr3_hand_control/msg/PoseCoordinates'], 
            capture_output=True, 
            text=True, 
            timeout=10
        )
        
        if result.returncode == 0 and 'shoulder_position' in result.stdout:
            print("✅ PoseCoordinates message definition found")
            
            # Test other messages
            messages_to_test = [
                'cr3_hand_control/msg/PoseTrackingStatus',
                'cr3_hand_control/msg/DebugInfo'
            ]
            
            all_good = True
            for msg in messages_to_test:
                result = subprocess.run(
                    ['ros2', 'interface', 'show', msg], 
                    capture_output=True, 
                    text=True, 
                    timeout=10
                )
                if result.returncode == 0:
                    print(f"✅ {msg} message found")
                else:
                    print(f"❌ {msg} message not found")
                    all_good = False
            
            return all_good
        else:
            print("❌ PoseCoordinates message not found")
            return False
            
    except Exception as e:
        print(f"❌ Message test failed: {e}")
        return False

def test_node_syntax():
    """Test Python syntax of the pose recognition node"""
    print("Testing node syntax...")
    
    try:
        # Test Python syntax without importing external dependencies
        node_path = "/home/andrewlh/CR3_OpenCV_Controls/ros2_package/src/cr3_hand_control/pose_recognition_node.py"
        
        if os.path.exists(node_path):
            # Test syntax compilation
            with open(node_path, 'r') as f:
                source = f.read()
            
            # Try to compile the source
            compile(source, node_path, 'exec')
            print("✅ Node syntax is valid")
            return True
        else:
            print(f"❌ Node file not found: {node_path}")
            return False
            
    except SyntaxError as e:
        print(f"❌ Syntax error in node: {e}")
        return False
    except Exception as e:
        print(f"❌ Node syntax test failed: {e}")
        return False

def main():
    """Main test function"""
    print("=" * 60)
    print("CR3 Pose Recognition ROS2 Integration Test")
    print("=" * 60)
    
    tests = [
        ("ROS2 Package", test_ros2_package),
        ("Launch Files", test_launch_files),
        ("Custom Messages", test_messages),
        ("Node Syntax", test_node_syntax),
    ]
    
    passed = 0
    total = len(tests)
    
    for test_name, test_func in tests:
        print(f"\n[{passed + 1}/{total}] {test_name}")
        print("-" * 40)
        
        try:
            if test_func():
                passed += 1
            else:
                print(f"Test {test_name} failed!")
        except Exception as e:
            print(f"Test {test_name} crashed: {e}")
    
    print("\n" + "=" * 60)
    print(f"Test Results: {passed}/{total} passed")
    
    if passed == total:
        print("🎉 ROS2 integration tests passed!")
        print("\nPhase 4 implementation is ready for testing with:")
        print("1. Install MediaPipe: pip install mediapipe (in virtual env)")
        print("2. Connect camera")
        print("3. Run: ros2 launch cr3_hand_control pose_recognition_test.launch.py")
    else:
        print("⚠️  Some integration tests failed. Please fix the issues.")
    
    print("=" * 60)
    return passed == total

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
