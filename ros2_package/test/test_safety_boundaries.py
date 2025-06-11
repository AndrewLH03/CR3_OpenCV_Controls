#!/usr/bin/env python3
"""
Safety System Tests for Phase 3
Tests workspace boundaries, emergency stops, and safety alerts.
"""

import unittest
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Point
from cr3_hand_control.msg import RobotStatus, SafetyAlert
from cr3_hand_control.srv import EmergencyStop


class TestSafetySystem(unittest.TestCase):
    """Test cases for the safety system"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment"""
        rclpy.init()
        cls.test_node = Node('safety_test_node')
        
        # Publishers for test data
        cls.robot_status_pub = cls.test_node.create_publisher(
            RobotStatus, '/robot_status', 10)
        
        # Subscribers for safety alerts
        cls.safety_alerts = []
        cls.safety_alert_sub = cls.test_node.create_subscription(
            SafetyAlert, '/safety_alerts',
            cls._safety_alert_callback, 10)
        
        # Service client for emergency stop
        cls.emergency_stop_client = cls.test_node.create_client(
            EmergencyStop, '/emergency_stop')
        
        # Wait for services to be available
        cls.emergency_stop_client.wait_for_service(timeout_sec=5.0)
        
        # Allow time for connections
        time.sleep(1.0)
    
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment"""
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def _safety_alert_callback(cls, msg):
        """Callback to collect safety alerts"""
        cls.safety_alerts.append(msg)
    
    def setUp(self):
        """Reset test state before each test"""
        self.safety_alerts.clear()
        
        # Spin node to process any pending messages
        rclpy.spin_once(self.test_node, timeout_sec=0.1)
    
    def _publish_robot_position(self, x, y, z):
        """Helper to publish robot position"""
        status = RobotStatus()
        status.connected = True
        status.moving = False
        status.enabled = True
        status.current_position = [x, y, z, 0.0, 0.0, 0.0]  # x, y, z, rx, ry, rz
        status.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        status.current_velocity = 10.0
        status.last_error = ""
        status.timestamp = self.test_node.get_clock().now().to_msg()
        
        self.robot_status_pub.publish(status)
        
        # Allow time for processing
        time.sleep(0.1)
        rclpy.spin_once(self.test_node, timeout_sec=0.1)
    
    def test_safe_position(self):
        """Test that safe positions don't trigger alerts"""
        # Publish position in safe zone (center of workspace)
        self._publish_robot_position(0.0, 0.0, 300.0)
        
        # Check that no danger alerts were generated
        danger_alerts = [alert for alert in self.safety_alerts if alert.level >= 2]
        self.assertEqual(len(danger_alerts), 0, "Safe position triggered danger alert")
    
    def test_boundary_approach_warning(self):
        """Test that approaching boundaries triggers warnings"""
        # Publish position near boundary (approach zone)
        self._publish_robot_position(360.0, 0.0, 300.0)  # Near x_max
        
        # Allow time for safety monitoring
        time.sleep(0.2)
        rclpy.spin_once(self.test_node, timeout_sec=0.1)
        
        # Check for approach warnings
        approach_alerts = [alert for alert in self.safety_alerts 
                          if alert.alert_type == "boundary"]
        self.assertGreater(len(approach_alerts), 0, "Boundary approach didn't trigger alert")
    
    def test_boundary_violation_danger(self):
        """Test that boundary violations trigger danger alerts"""
        # Publish position very close to boundary (danger zone)
        self._publish_robot_position(395.0, 0.0, 300.0)  # Very close to x_max
        
        # Allow time for safety monitoring
        time.sleep(0.2)
        rclpy.spin_once(self.test_node, timeout_sec=0.1)
        
        # Check for danger alerts
        danger_alerts = [alert for alert in self.safety_alerts if alert.level >= 2]
        self.assertGreater(len(danger_alerts), 0, "Boundary violation didn't trigger danger alert")
    
    def test_emergency_stop_service(self):
        """Test emergency stop service"""
        # Create emergency stop request
        request = EmergencyStop.Request()
        request.stop_type = "emergency"
        request.reason = "Test emergency stop"
        
        # Call service
        future = self.emergency_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        
        # Check response
        self.assertTrue(future.done(), "Emergency stop service call timed out")
        response = future.result()
        self.assertTrue(response.success, f"Emergency stop failed: {response.message}")
    
    def test_emergency_stop_reset(self):
        """Test emergency stop reset"""
        # First trigger emergency stop
        request = EmergencyStop.Request()
        request.stop_type = "emergency"
        request.reason = "Test emergency stop"
        future = self.emergency_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        
        # Then reset
        request.stop_type = "reset"
        request.reason = "Test reset"
        future = self.emergency_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        
        # Check response
        response = future.result()
        self.assertTrue(response.success, f"Emergency stop reset failed: {response.message}")


def main():
    """Run safety system tests"""
    unittest.main()


if __name__ == '__main__':
    main()
