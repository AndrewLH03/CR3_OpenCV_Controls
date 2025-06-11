#!/usr/bin/env python3
"""
Phase 3 Message Tests
Tests that all Phase 3 message types can be constructed and used correctly.
"""

import unittest
import rclpy
from rclpy.node import Node
import time
from geometry_msgs.msg import Point
from cr3_hand_control.msg import (
    RobotStatus, SafetyAlert, CoordinateTransform,
    BasicCommand, HandPosition, CombinedData
)
from cr3_hand_control.srv import EmergencyStop, CalibrationService


class TestPhase3Messages(unittest.TestCase):
    """Test that all Phase 3 messages can be constructed properly"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment"""
        rclpy.init()
        cls.test_node = Node('message_test_node')
    
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
        alert.level = 2  # DANGER level
        alert.alert_type = "boundary"
        alert.description = "Approaching workspace boundary"
        
        # Create a Point for location
        location = Point()
        location.x = 350.0
        location.y = 0.0
        location.z = 300.0
        alert.location = location
        
        alert.distance_to_boundary = 25.0
        alert.recommended_speed = 10.0
        alert.timestamp = self.test_node.get_clock().now().to_msg()
        
        # Verify fields
        self.assertEqual(alert.level, 2)
        self.assertEqual(alert.alert_type, "boundary")
        self.assertEqual(alert.location.x, 350.0)
        self.assertEqual(alert.distance_to_boundary, 25.0)
    
    def test_coordinate_transform_message(self):
        """Test CoordinateTransform message construction"""
        transform = CoordinateTransform()
        transform.source_frame = "camera_link"
        transform.target_frame = "base_link"
        transform.transform_matrix = [1.0, 0.0, 0.0, 100.0,
                                    0.0, 1.0, 0.0, 200.0,
                                    0.0, 0.0, 1.0, 300.0,
                                    0.0, 0.0, 0.0, 1.0]
        transform.accuracy = 0.5
        transform.is_valid = True
        transform.timestamp = self.test_node.get_clock().now().to_msg()
        
        # Verify fields
        self.assertEqual(transform.source_frame, "camera_link")
        self.assertEqual(transform.target_frame, "base_link")
        self.assertEqual(len(transform.transform_matrix), 16)
        self.assertEqual(transform.accuracy, 0.5)
        self.assertTrue(transform.is_valid)
    
    def test_basic_command_message(self):
        """Test BasicCommand message construction"""
        command = BasicCommand()
        command.command_type = "Move"
        command.position = [100.0, 200.0, 300.0, 0.0, 0.0, 0.0]
        command.velocity = 50.0
        command.relative = False
        
        # Verify fields
        self.assertEqual(command.command_type, "Move")
        self.assertEqual(len(command.position), 6)
        self.assertEqual(command.velocity, 50.0)
        self.assertFalse(command.relative)
    
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
    
    def test_hand_position_message(self):
        """Test HandPosition message construction"""
        hand_pos = HandPosition()
        hand_pos.hand_detected = True
        hand_pos.confidence = 0.95
        
        # Create landmarks
        for i in range(21):  # 21 hand landmarks
            landmark = Point()
            landmark.x = float(i * 10)
            landmark.y = float(i * 5)
            landmark.z = float(i * 2)
            hand_pos.landmarks.append(landmark)
        
        hand_pos.timestamp = self.test_node.get_clock().now().to_msg()
        
        # Verify fields
        self.assertTrue(hand_pos.hand_detected)
        self.assertEqual(hand_pos.confidence, 0.95)
        self.assertEqual(len(hand_pos.landmarks), 21)
        self.assertEqual(hand_pos.landmarks[5].x, 50.0)
    
    def test_combined_data_message(self):
        """Test CombinedData message construction"""
        combined = CombinedData()
        
        # Add robot status
        robot_status = RobotStatus()
        robot_status.connected = True
        robot_status.current_position = [0.0, 0.0, 300.0, 0.0, 0.0, 0.0]
        robot_status.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        robot_status.current_velocity = 0.0
        combined.robot_status = robot_status
        
        # Add hand position
        hand_pos = HandPosition()
        hand_pos.hand_detected = True
        hand_pos.confidence = 0.8
        combined.hand_position = hand_pos
        
        combined.timestamp = self.test_node.get_clock().now().to_msg()
        
        # Verify fields
        self.assertTrue(combined.robot_status.connected)
        self.assertTrue(combined.hand_position.hand_detected)
        self.assertEqual(combined.hand_position.confidence, 0.8)
    
    def test_message_publishing(self):
        """Test that messages can be published and received"""
        # Create publishers
        status_pub = self.test_node.create_publisher(RobotStatus, '/test_robot_status', 10)
        alert_pub = self.test_node.create_publisher(SafetyAlert, '/test_safety_alerts', 10)
        
        # Storage for received messages
        received_status = []
        received_alerts = []
        
        def status_callback(msg):
            received_status.append(msg)
        
        def alert_callback(msg):
            received_alerts.append(msg)
        
        # Create subscribers
        status_sub = self.test_node.create_subscription(
            RobotStatus, '/test_robot_status', status_callback, 10)
        alert_sub = self.test_node.create_subscription(
            SafetyAlert, '/test_safety_alerts', alert_callback, 10)
        
        # Allow time for connections
        time.sleep(0.5)
        
        # Publish test messages
        status = RobotStatus()
        status.connected = True
        status.current_position = [123.0, 456.0, 789.0, 0.0, 0.0, 0.0]
        status.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        status.current_velocity = 25.0
        status_pub.publish(status)
        
        alert = SafetyAlert()
        alert.level = 1
        alert.alert_type = "test"
        alert.description = "Test alert"
        alert_pub.publish(alert)
        
        # Spin to process messages
        for _ in range(10):
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            time.sleep(0.1)
        
        # Verify messages were received
        self.assertGreater(len(received_status), 0, "No RobotStatus messages received")
        self.assertGreater(len(received_alerts), 0, "No SafetyAlert messages received")
        
        # Verify message content
        if received_status:
            self.assertTrue(received_status[0].connected)
            self.assertEqual(received_status[0].current_position[0], 123.0)
        
        if received_alerts:
            self.assertEqual(received_alerts[0].level, 1)
            self.assertEqual(received_alerts[0].alert_type, "test")


def main():
    """Run Phase 3 message tests"""
    unittest.main()


if __name__ == '__main__':
    main()
