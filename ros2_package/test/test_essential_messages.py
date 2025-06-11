#!/usr/bin/env python3
"""
Phase 3 Essential Message Tests
Tests basic message construction for Phase 3 system validation.
"""

import unittest
import rclpy
from rclpy.node import Node
from cr3_hand_control.msg import RobotStatus, SafetyAlert, CoordinateTransform
from cr3_hand_control.srv import EmergencyStop


class TestEssentialMessages(unittest.TestCase):
    """Test essential Phase 3 message construction"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment"""
        rclpy.init()
        cls.test_node = Node('essential_message_test_node')
    
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment"""
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_robot_status_message(self):
        """Test RobotStatus message construction"""
        status = RobotStatus()
        status.connected = True
        status.moving = False
        status.enabled = True
        status.current_position = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        status.current_joint_angles = [0.0, 30.0, 45.0, 0.0, 0.0, 0.0]
        status.current_velocity = 50.0
        status.last_error = "No error"
        status.timestamp = self.test_node.get_clock().now().to_msg()
        
        # Verify fields are set correctly
        self.assertTrue(status.connected)
        self.assertFalse(status.moving)
        self.assertEqual(len(status.current_position), 6)
        self.assertEqual(status.current_position[0], 100.0)
        self.assertEqual(status.current_velocity, 50.0)
    
    def test_safety_alert_message(self):
        """Test SafetyAlert message construction"""
        alert = SafetyAlert()
        alert.level = 2
        alert.alert_type = "boundary"
        alert.description = "Approaching workspace boundary"
        alert.location = [350.0, 0.0, 300.0]
        alert.distance_to_boundary = 50.0
        alert.recommended_speed = 0.1
        alert.timestamp = self.test_node.get_clock().now().to_msg()
        
        # Verify fields
        self.assertEqual(alert.level, 2)
        self.assertEqual(alert.alert_type, "boundary")
        self.assertEqual(len(alert.location), 3)
        self.assertEqual(alert.distance_to_boundary, 50.0)
    
    def test_emergency_stop_service(self):
        """Test EmergencyStop service message construction"""
        # Test request
        request = EmergencyStop.Request()
        request.stop_type = "emergency"
        request.reason = "Test emergency stop"
        
        self.assertEqual(request.stop_type, "emergency")
        self.assertEqual(request.reason, "Test emergency stop")
        
        # Test response
        response = EmergencyStop.Response()
        response.success = True
        response.message = "Emergency stop activated"
        response.stop_time = 0.01
        
        self.assertTrue(response.success)
        self.assertEqual(response.message, "Emergency stop activated")
        self.assertEqual(response.stop_time, 0.01)


def main():
    """Run essential message tests"""
    unittest.main()


if __name__ == '__main__':
    main()
