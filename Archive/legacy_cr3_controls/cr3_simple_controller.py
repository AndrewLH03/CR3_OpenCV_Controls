#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CR3 Simple Robot Controller

A simplified robot controller that demonstrates basic CR3 robot movements
without the complex abstraction layers that caused issues in Hand_Tracking.

This script provides:
- Direct TCP socket communication with CR3 robot
- Simple movement to packing position and back
- Clear error handling and logging
- Minimal dependencies for maximum reliability

Usage:
    python cr3_simple_controller.py --robot-ip 192.168.1.6
"""

import socket
import time
import argparse
import sys
from typing import Tuple, List


class SimpleCR3Controller:
    """
    Simple, direct CR3 robot controller using TCP socket communication.
    
    No complex abstractions, just basic socket communication for reliable robot control.
    """
    
    def __init__(self, robot_ip: str = "192.168.1.6", dashboard_port: int = 29999):
        """
        Initialize the simple CR3 controller.
        
        Args:
            robot_ip: IP address of the CR3 robot
            dashboard_port: TCP port for dashboard commands (usually 29999)
        """
        self.robot_ip = robot_ip
        self.dashboard_port = dashboard_port
        self.socket = None
        self.connected = False
        
        # Define safe positions
        self.initial_position = [250.0, 0.0, 200.0, 0.0, 0.0, 0.0]  # Safe home position
        self.packing_position = [200.0, -100.0, 150.0, 0.0, 0.0, 0.0]  # Packing position
        
    def connect(self) -> Tuple[bool, str]:
        """
        Connect to the CR3 robot via TCP socket.
        
        Returns:
            (success, message): Connection result
        """
        try:
            # Create socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)  # 10 second timeout
            
            print(f"[INFO] Connecting to CR3 robot at {self.robot_ip}:{self.dashboard_port}")
            
            # Connect to robot
            self.socket.connect((self.robot_ip, self.dashboard_port))
            
            # Wait for initial response
            response = self._receive_data()
            print(f"[INFO] Initial response: {response}")
            
            self.connected = True
            return True, f"Connected successfully to {self.robot_ip}"
            
        except socket.timeout:
            return False, "Connection timeout - check robot IP and network"
        except ConnectionRefusedError:
            return False, "Connection refused - check robot is powered and IP is correct"
        except Exception as e:
            return False, f"Connection failed: {str(e)}"
    
    def disconnect(self) -> None:
        """Disconnect from the robot."""
        if self.socket:
            try:
                self.socket.close()
            except:
                pass
            self.socket = None
        self.connected = False
        print("[INFO] Disconnected from robot")
    
    def _send_command(self, command: str) -> Tuple[bool, str]:
        """
        Send a command to the robot and get response.
        
        Args:
            command: Robot command string
            
        Returns:
            (success, response): Command result
        """
        if not self.connected or not self.socket:
            return False, "Not connected to robot"
        
        try:
            # Send command
            command_bytes = (command + "\n").encode('utf-8')
            self.socket.send(command_bytes)
            
            # Receive response
            response = self._receive_data()
            
            print(f"[CMD] {command} -> {response}")
            
            # Check if command was successful (response should start with '0' for success)
            if response.startswith('0'):
                return True, response
            else:
                return False, f"Command failed: {response}"
                
        except Exception as e:
            return False, f"Command error: {str(e)}"
    
    def _receive_data(self) -> str:
        """Receive data from robot socket."""
        try:
            data = self.socket.recv(1024).decode('utf-8').strip()
            return data
        except Exception as e:
            return f"Receive error: {str(e)}"
    
    def enable_robot(self) -> Tuple[bool, str]:
        """Enable the robot for movement."""
        print("[INFO] Enabling robot...")
        return self._send_command("EnableRobot()")
    
    def disable_robot(self) -> Tuple[bool, str]:
        """Disable the robot."""
        print("[INFO] Disabling robot...")
        return self._send_command("DisableRobot()")
    
    def clear_error(self) -> Tuple[bool, str]:
        """Clear any robot errors."""
        print("[INFO] Clearing robot errors...")
        return self._send_command("ClearError()")
    
    def get_pose(self) -> Tuple[bool, str]:
        """Get current robot pose."""
        return self._send_command("GetPose()")
    
    def move_to_position(self, position: List[float], speed: float = 50.0) -> Tuple[bool, str]:
        """
        Move robot to specified position using MovL (linear movement).
        
        Args:
            position: [X, Y, Z, Rx, Ry, Rz] position
            speed: Movement speed percentage (1-100)
            
        Returns:
            (success, response): Movement result
        """
        # Format position for MovL command
        x, y, z, rx, ry, rz = position
        command = f"MovL({x},{y},{z},{rx},{ry},{rz})"
        
        print(f"[MOVE] Moving to position: X={x}, Y={y}, Z={z}, Rx={rx}, Ry={ry}, Rz={rz}")
        
        success, response = self._send_command(command)
        
        if success:
            # Wait for movement to complete
            print("[INFO] Waiting for movement to complete...")
            time.sleep(3.0)  # Basic wait - in production, should monitor robot status
        
        return success, response
    
    def robot_mode(self) -> Tuple[bool, str]:
        """Get robot mode."""
        return self._send_command("RobotMode()")
    
    def run_demo_sequence(self) -> bool:
        """
        Run a complete demo sequence: move to packing position and back to initial position.
        
        Returns:
            bool: True if sequence completed successfully
        """
        print("\n" + "="*50)
        print("CR3 ROBOT DEMO SEQUENCE STARTING")
        print("="*50)
        
        # Step 1: Clear errors and enable robot
        success, response = self.clear_error()
        if not success:
            print(f"[ERROR] Failed to clear errors: {response}")
            return False
        
        success, response = self.enable_robot()
        if not success:
            print(f"[ERROR] Failed to enable robot: {response}")
            return False
        
        # Step 2: Check robot mode
        success, response = self.robot_mode()
        if success:
            print(f"[INFO] Robot mode: {response}")
        
        # Step 3: Get initial position
        success, response = self.get_pose()
        if success:
            print(f"[INFO] Current position: {response}")
        
        # Step 4: Move to packing position
        print(f"\n[SEQUENCE] Moving to packing position...")
        success, response = self.move_to_position(self.packing_position)
        if not success:
            print(f"[ERROR] Failed to move to packing position: {response}")
            return False
        
        print("[SUCCESS] Reached packing position!")
        
        # Step 5: Wait at packing position
        print("[INFO] Holding at packing position for 2 seconds...")
        time.sleep(2.0)
        
        # Step 6: Return to initial position
        print(f"\n[SEQUENCE] Returning to initial position...")
        success, response = self.move_to_position(self.initial_position)
        if not success:
            print(f"[ERROR] Failed to return to initial position: {response}")
            return False
        
        print("[SUCCESS] Returned to initial position!")
        
        # Step 7: Final position check
        success, response = self.get_pose()
        if success:
            print(f"[INFO] Final position: {response}")
        
        print("\n" + "="*50)
        print("CR3 ROBOT DEMO SEQUENCE COMPLETED SUCCESSFULLY")
        print("="*50)
        
        return True


def main():
    """Main function to run the CR3 simple controller demo."""
    parser = argparse.ArgumentParser(description="Simple CR3 Robot Controller Demo")
    parser.add_argument("--robot-ip", default="192.168.1.6", 
                       help="IP address of the CR3 robot (default: 192.168.1.6)")
    parser.add_argument("--port", type=int, default=29999,
                       help="TCP port for robot dashboard (default: 29999)")
    args = parser.parse_args()
    
    # Create controller
    controller = SimpleCR3Controller(args.robot_ip, args.port)
    
    try:
        # Connect to robot
        success, message = controller.connect()
        if not success:
            print(f"[ERROR] Connection failed: {message}")
            print("\nTroubleshooting:")
            print("1. Check robot IP address and network connection")
            print("2. Ensure robot is powered on and network is accessible")
            print("3. Verify firewall settings allow connection to port 29999")
            sys.exit(1)
        
        print(f"[SUCCESS] {message}")
        
        # Run demo sequence
        success = controller.run_demo_sequence()
        
        if success:
            print("\n[DEMO] Demo completed successfully!")
            return_code = 0
        else:
            print("\n[DEMO] Demo failed - check error messages above")
            return_code = 1
        
    except KeyboardInterrupt:
        print("\n[INFO] Demo interrupted by user")
        return_code = 1
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {str(e)}")
        return_code = 1
    finally:
        # Always disconnect
        controller.disconnect()
    
    sys.exit(return_code)


if __name__ == "__main__":
    main()
