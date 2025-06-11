#!/usr/bin/env python3
"""
Transform Manager for CR3 Robot System
Manages coordinate transformations between camera and robot frames using TF2.
"""

import rclpy
from rclpy.node import Node
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, Point, PointStamped
from cr3_hand_control.msg import CoordinateTransform
import numpy as np
import yaml
import os


class TransformManager(Node):
    """
    Manages coordinate transformations between camera and robot coordinate systems.
    Provides transformation services and validates transformation accuracy.
    """
    
    def __init__(self):
        super().__init__('transform_manager')
        
        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameter('calibration.camera_frame', 'camera_link')
        self.declare_parameter('calibration.robot_base_frame', 'base_link')
        self.declare_parameter('calibration.world_frame', 'world')
        self.declare_parameter('calibration.transforms.update_frequency', 30.0)
        self.declare_parameter('calibration.transforms.timeout', 1.0)
        
        self.camera_frame = self.get_parameter('calibration.camera_frame').value
        self.robot_frame = self.get_parameter('calibration.robot_base_frame').value
        self.world_frame = self.get_parameter('calibration.world_frame').value
        self.update_frequency = self.get_parameter('calibration.transforms.update_frequency').value
        self.transform_timeout = self.get_parameter('calibration.transforms.timeout').value
        
        # Publishers
        self.coordinate_transform_pub = self.create_publisher(
            CoordinateTransform,
            '/coordinate_transforms',
            10
        )
        
        # Current transformation
        self.current_transform = None
        self.transform_accuracy = 0.0
        self.transform_valid = False
        
        # Create timer for periodic transform updates
        self.transform_timer = self.create_timer(
            1.0 / self.update_frequency,
            self.update_transforms
        )
        
        self.get_logger().info(f"Transform Manager initialized")
        self.get_logger().info(f"Managing transforms: {self.camera_frame} <-> {self.robot_frame}")
    
    def update_transforms(self):
        """Periodically update and publish coordinate transformations"""
        try:
            # Get transform from camera to robot base
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            
            self.current_transform = transform
            self.transform_valid = True
            
            # Calculate transformation accuracy (simplified)
            self.transform_accuracy = self._calculate_transform_accuracy(transform)
            
            # Publish transform message
            self._publish_coordinate_transform(transform)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            if self.transform_valid:  # Only log once when transform becomes invalid
                self.get_logger().warn(f"Transform lookup failed: {e}")
                self.transform_valid = False
    
    def _calculate_transform_accuracy(self, transform):
        """
        Calculate estimated accuracy of the transformation.
        In a real system, this would involve more sophisticated validation.
        """
        # For now, return a fixed accuracy estimate
        # In practice, this would be based on calibration residuals
        return 2.0  # mm
    
    def _publish_coordinate_transform(self, transform):
        """Publish coordinate transformation message"""
        coord_msg = CoordinateTransform()
        coord_msg.source_frame = self.camera_frame
        coord_msg.target_frame = self.robot_frame
        coord_msg.transform = transform.transform
        coord_msg.accuracy = self.transform_accuracy
        coord_msg.is_valid = self.transform_valid
        coord_msg.timestamp = self.get_clock().now().to_msg()
        
        self.coordinate_transform_pub.publish(coord_msg)
    
    def transform_point(self, point, source_frame, target_frame):
        """
        Transform a point from source frame to target frame.
        
        Args:
            point: geometry_msgs/Point to transform
            source_frame: Source coordinate frame
            target_frame: Target coordinate frame
            
        Returns:
            Transformed point or None if transformation fails
        """
        try:
            # Create PointStamped message
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point = point
            
            # Transform the point
            transformed_point = self.tf_buffer.transform(
                point_stamped,
                target_frame,
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            
            return transformed_point.point
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to transform point: {e}")
            return None
    
    def camera_to_robot(self, camera_point):
        """Transform point from camera frame to robot frame"""
        return self.transform_point(camera_point, self.camera_frame, self.robot_frame)
    
    def robot_to_camera(self, robot_point):
        """Transform point from robot frame to camera frame"""
        return self.transform_point(robot_point, self.robot_frame, self.camera_frame)
    
    def get_transform_matrix(self, source_frame, target_frame):
        """
        Get transformation matrix between two frames.
        
        Returns:
            4x4 numpy transformation matrix or None if unavailable
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.transform_timeout)
            )
            
            # Convert to transformation matrix
            return self._transform_to_matrix(transform.transform)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to get transform matrix: {e}")
            return None
    
    def _transform_to_matrix(self, transform):
        """Convert geometry_msgs/Transform to 4x4 numpy matrix"""
        # Translation
        tx, ty, tz = transform.translation.x, transform.translation.y, transform.translation.z
        
        # Rotation (quaternion to rotation matrix)
        qx, qy, qz, qw = transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w
        
        # Convert quaternion to rotation matrix
        rotation_matrix = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)]
        ])
        
        # Create 4x4 transformation matrix
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = rotation_matrix
        transform_matrix[:3, 3] = [tx, ty, tz]
        
        return transform_matrix
    
    def is_transform_available(self, source_frame, target_frame):
        """Check if transform between frames is available"""
        try:
            self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            return True
        except:
            return False
    
    def get_transform_info(self):
        """Get current transformation information"""
        return {
            'camera_frame': self.camera_frame,
            'robot_frame': self.robot_frame,
            'transform_valid': self.transform_valid,
            'accuracy': self.transform_accuracy,
            'last_update': self.get_clock().now().to_msg()
        }


def main(args=None):
    rclpy.init(args=args)
    
    transform_manager = TransformManager()
    
    try:
        rclpy.spin(transform_manager)
    except KeyboardInterrupt:
        pass
    finally:
        transform_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
