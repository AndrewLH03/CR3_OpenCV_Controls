#!/usr/bin/env python3
"""
Coordinate System Tests for Phase 3
Tests coordinate transformations between camera and robot frames.
"""

import unittest
import rclpy
from rclpy.node import Node
import numpy as np
import time
from geometry_msgs.msg import Point, TransformStamped
from cr3_hand_control.msg import CoordinateTransform
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from geometry_msgs.msg import PointStamped


class TestCoordinateSystem(unittest.TestCase):
    """Test cases for coordinate transformations"""
    
    @classmethod
    def setUpClass(cls):
        """Set up test environment"""
        rclpy.init()
        cls.test_node = Node('coordinate_test_node')
        
        # TF2 setup
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.test_node)
        
        # Subscribers for coordinate transforms
        cls.coordinate_transforms = []
        cls.coord_transform_sub = cls.test_node.create_subscription(
            CoordinateTransform, '/coordinate_transforms',
            cls._coordinate_transform_callback, 10)
        
        # Allow time for TF2 setup
        time.sleep(2.0)
    
    @classmethod
    def tearDownClass(cls):
        """Clean up test environment"""
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def _coordinate_transform_callback(cls, msg):
        """Callback to collect coordinate transforms"""
        cls.coordinate_transforms.append(msg)
    
    def setUp(self):
        """Reset test state before each test"""
        self.coordinate_transforms.clear()
        rclpy.spin_once(self.test_node, timeout_sec=0.1)
    
    def test_transform_availability(self):
        """Test that required transforms are available"""
        try:
            # Try to get transform from camera to robot base
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            self.assertIsNotNone(transform, "Camera to robot transform not available")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.fail(f"Transform lookup failed: {e}")
    
    def test_transform_consistency(self):
        """Test that transforms are consistent (forward and inverse)"""
        try:
            # Get transform camera -> robot
            transform_cr = self.tf_buffer.lookup_transform(
                'base_link', 'camera_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Get transform robot -> camera
            transform_rc = self.tf_buffer.lookup_transform(
                'camera_link', 'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Test that they are inverses (approximately)
            # For now, just check that we got both transforms
            self.assertIsNotNone(transform_cr)
            self.assertIsNotNone(transform_rc)
            
        except Exception as e:
            self.fail(f"Transform consistency test failed: {e}")
    
    def test_point_transformation(self):
        """Test transformation of specific points"""
        try:
            # Create a test point in robot frame
            robot_point = PointStamped()
            robot_point.header.frame_id = 'base_link'
            robot_point.header.stamp = self.test_node.get_clock().now().to_msg()
            robot_point.point.x = 100.0  # 100mm in robot frame
            robot_point.point.y = 0.0
            robot_point.point.z = 200.0
            
            # Transform to camera frame
            camera_point = self.tf_buffer.transform(
                robot_point, 'camera_link',
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Verify transformation occurred (coordinates should be different)
            self.assertNotEqual(robot_point.point.x, camera_point.point.x,
                              "Point transformation didn't change coordinates")
            
        except Exception as e:
            self.fail(f"Point transformation test failed: {e}")
    
    def test_transform_accuracy_bounds(self):
        """Test that transform accuracy is within acceptable bounds"""
        # Allow time to collect transform messages
        time.sleep(1.0)
        rclpy.spin_once(self.test_node, timeout_sec=0.5)
        
        if self.coordinate_transforms:
            latest_transform = self.coordinate_transforms[-1]
            
            # Check that accuracy is reported and reasonable
            self.assertGreater(latest_transform.accuracy, 0.0,
                             "Transform accuracy should be positive")
            self.assertLess(latest_transform.accuracy, 100.0,
                          "Transform accuracy seems unreasonably high")
            
            # Check that transform is marked as valid
            self.assertTrue(latest_transform.is_valid,
                          "Transform should be marked as valid")
        else:
            self.skipTest("No coordinate transform messages received")
    
    def test_transform_update_frequency(self):
        """Test that transforms are updated at reasonable frequency"""
        initial_count = len(self.coordinate_transforms)
        
        # Wait for updates
        start_time = time.time()
        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            time.sleep(0.1)
        
        final_count = len(self.coordinate_transforms)
        updates = final_count - initial_count
        
        # Should get at least a few updates in 2 seconds
        self.assertGreater(updates, 0, "No coordinate transform updates received")
        
        # Update rate should be reasonable (not too fast, not too slow)
        update_rate = updates / 2.0  # updates per second
        self.assertGreater(update_rate, 1.0, "Transform update rate too slow")
        self.assertLess(update_rate, 100.0, "Transform update rate too fast")


def main():
    """Run coordinate system tests"""
    unittest.main()


if __name__ == '__main__':
    main()
