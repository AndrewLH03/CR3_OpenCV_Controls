#!/usr/bin/env python3
"""
Calibration Node for CR3 Robot System
Interactive calibration procedures for camera-robot coordinate alignment.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Point, TransformStamped
from cr3_hand_control.msg import CoordinateTransform, BasicCommand, RobotStatus
from tf2_ros import StaticTransformBroadcaster
import tf2_geometry_msgs
import yaml
import os
from scipy.spatial.transform import Rotation


class CalibrationNode(Node):
    """
    Interactive calibration system for establishing camera-robot coordinate transformations.
    Guides user through calibration process and validates accuracy.
    """
    
    def __init__(self):
        super().__init__('calibration_node')
        
        # Parameters
        self._declare_calibration_parameters()
        self._load_calibration_config()
        
        # Publishers
        self.calibration_command_pub = self.create_publisher(
            BasicCommand,
            '/calibration_commands',
            10
        )
        
        self.coordinate_transform_pub = self.create_publisher(
            CoordinateTransform,
            '/calibration_transforms',
            10
        )
        
        # Static transform broadcaster for calibration results
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Subscribers
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/robot_status',
            self.robot_status_callback,
            10
        )
        
        # Calibration state
        self.calibration_points = []
        self.robot_points = []
        self.camera_points = []
        self.current_step = 0
        self.calibration_active = False
        self.transform_matrix = None
        self.calibration_accuracy = 0.0
        
        # Generate calibration grid
        self._generate_calibration_points()
        
        self.get_logger().info(f"Calibration Node initialized with {len(self.calibration_points)} calibration points")
    
    def _declare_calibration_parameters(self):
        """Declare calibration parameters"""
        self.declare_parameter('calibration.camera_frame', 'camera_link')
        self.declare_parameter('calibration.robot_base_frame', 'base_link')
        self.declare_parameter('calibration.calibration_points.num_points', 9)
        self.declare_parameter('calibration.calibration_points.workspace_coverage', 0.8)
        self.declare_parameter('calibration.accuracy_requirements.position_tolerance', 2.0)
        self.declare_parameter('calibration.validation.test_points', 5)
        self.declare_parameter('calibration.validation.max_error', 3.0)
        
        # Workspace boundaries for calibration point generation
        self.declare_parameter('robot.workspace.x_min', -400.0)
        self.declare_parameter('robot.workspace.x_max', 400.0)
        self.declare_parameter('robot.workspace.y_min', -400.0)
        self.declare_parameter('robot.workspace.y_max', 400.0)
        self.declare_parameter('robot.workspace.z_min', 100.0)
        self.declare_parameter('robot.workspace.z_max', 500.0)
    
    def _load_calibration_config(self):
        """Load calibration configuration"""
        self.camera_frame = self.get_parameter('calibration.camera_frame').value
        self.robot_frame = self.get_parameter('calibration.robot_base_frame').value
        self.num_points = self.get_parameter('calibration.calibration_points.num_points').value
        self.workspace_coverage = self.get_parameter('calibration.calibration_points.workspace_coverage').value
        self.position_tolerance = self.get_parameter('calibration.accuracy_requirements.position_tolerance').value
        self.test_points = self.get_parameter('calibration.validation.test_points').value
        self.max_error = self.get_parameter('calibration.validation.max_error').value
        
        # Workspace boundaries
        self.x_min = self.get_parameter('robot.workspace.x_min').value
        self.x_max = self.get_parameter('robot.workspace.x_max').value
        self.y_min = self.get_parameter('robot.workspace.y_min').value
        self.y_max = self.get_parameter('robot.workspace.y_max').value
        self.z_min = self.get_parameter('robot.workspace.z_min').value
        self.z_max = self.get_parameter('robot.workspace.z_max').value
    
    def _generate_calibration_points(self):
        """Generate grid of calibration points within workspace"""
        # Calculate workspace dimensions with coverage factor
        x_range = (self.x_max - self.x_min) * self.workspace_coverage
        y_range = (self.y_max - self.y_min) * self.workspace_coverage
        z_range = (self.z_max - self.z_min) * self.workspace_coverage
        
        # Center the reduced workspace
        x_center = (self.x_max + self.x_min) / 2
        y_center = (self.y_max + self.y_min) / 2
        z_center = (self.z_max + self.z_min) / 2
        
        # Generate 3D grid (assuming square root gives grid dimensions)
        grid_size = int(round(self.num_points ** (1/3)))
        if grid_size ** 3 < self.num_points:
            grid_size += 1
        
        points = []
        for i in range(grid_size):
            for j in range(grid_size):
                for k in range(grid_size):
                    if len(points) >= self.num_points:
                        break
                        
                    x = x_center + (i / (grid_size - 1) - 0.5) * x_range
                    y = y_center + (j / (grid_size - 1) - 0.5) * y_range
                    z = z_center + (k / (grid_size - 1) - 0.5) * z_range
                    
                    point = Point()
                    point.x = x
                    point.y = y
                    point.z = z
                    points.append(point)
                    
                if len(points) >= self.num_points:
                    break
            if len(points) >= self.num_points:
                break
        
        self.calibration_points = points[:self.num_points]
        self.get_logger().info(f"Generated {len(self.calibration_points)} calibration points")
    
    def robot_status_callback(self, msg):
        """Handle robot status updates during calibration"""
        if self.calibration_active and self.current_step < len(self.calibration_points):
            # Check if robot has reached the calibration point
            target_point = self.calibration_points[self.current_step]
            current_point = msg.current_position
            
            # Calculate distance to target
            dx = current_point.x - target_point.x
            dy = current_point.y - target_point.y
            dz = current_point.z - target_point.z
            distance = np.sqrt(dx*dx + dy*dy + dz*dz)
            
            if distance < self.position_tolerance:
                self.get_logger().info(f"Robot reached calibration point {self.current_step + 1}/{len(self.calibration_points)}")
                self.get_logger().info("Please position camera to view robot and press Enter to record point...")
                
                # Wait for user input (in a real implementation, this would be handled differently)
                # For now, we'll simulate automatic point recording
                self._record_calibration_point(current_point)
    
    def start_calibration(self):
        """Start the calibration process"""
        if self.calibration_active:
            self.get_logger().warn("Calibration already in progress")
            return
        
        self.calibration_active = True
        self.current_step = 0
        self.robot_points.clear()
        self.camera_points.clear()
        
        self.get_logger().info("Starting calibration process...")
        self.get_logger().info(f"Will collect {len(self.calibration_points)} calibration points")
        
        # Move to first calibration point
        self._move_to_calibration_point(0)
    
    def _move_to_calibration_point(self, point_index):
        """Command robot to move to calibration point"""
        if point_index >= len(self.calibration_points):
            self._complete_calibration()
            return
        
        target_point = self.calibration_points[point_index]
        
        # Send movement command
        command = BasicCommand()
        command.command_type = "move_to_position"
        command.target_position = target_point
        command.speed = 50.0  # Slow movement for calibration
        command.timestamp = self.get_clock().now().to_msg()
        
        self.calibration_command_pub.publish(command)
        
        self.get_logger().info(f"Moving to calibration point {point_index + 1}/{len(self.calibration_points)}: "
                              f"[{target_point.x:.1f}, {target_point.y:.1f}, {target_point.z:.1f}]")
    
    def _record_calibration_point(self, robot_point):
        """Record a calibration point pair (robot + camera)"""
        # In a real implementation, this would capture the camera view
        # For now, we'll simulate camera point detection
        camera_point = self._simulate_camera_detection(robot_point)
        
        self.robot_points.append([robot_point.x, robot_point.y, robot_point.z])
        self.camera_points.append(camera_point)
        
        self.get_logger().info(f"Recorded calibration point {self.current_step + 1}: "
                              f"Robot[{robot_point.x:.1f}, {robot_point.y:.1f}, {robot_point.z:.1f}] "
                              f"Camera[{camera_point[0]:.1f}, {camera_point[1]:.1f}, {camera_point[2]:.1f}]")
        
        self.current_step += 1
        
        # Move to next point or complete calibration
        if self.current_step < len(self.calibration_points):
            self._move_to_calibration_point(self.current_step)
        else:
            self._complete_calibration()
    
    def _simulate_camera_detection(self, robot_point):
        """Simulate camera detection of robot position (for testing)"""
        # In reality, this would use computer vision to detect the robot position
        # For simulation, add some noise and a simple transformation
        noise = np.random.normal(0, 1, 3)  # 1mm standard deviation noise
        
        # Simple transform: rotate 90 degrees around Z and translate
        camera_x = -robot_point.y + noise[0]
        camera_y = robot_point.x + noise[1]
        camera_z = robot_point.z - 500 + noise[2]  # Camera 500mm above robot base
        
        return [camera_x, camera_y, camera_z]
    
    def _complete_calibration(self):
        """Complete calibration and calculate transformation"""
        self.calibration_active = False
        
        if len(self.robot_points) < 4:
            self.get_logger().error("Insufficient calibration points for transformation calculation")
            return
        
        # Calculate transformation matrix
        self.transform_matrix = self._calculate_transformation()
        
        if self.transform_matrix is not None:
            # Validate calibration accuracy
            self.calibration_accuracy = self._validate_calibration()
            
            # Publish transformation
            self._publish_calibration_result()
            
            # Save calibration to file
            self._save_calibration()
            
            self.get_logger().info(f"Calibration completed with accuracy: {self.calibration_accuracy:.2f}mm")
        else:
            self.get_logger().error("Failed to calculate transformation matrix")
    
    def _calculate_transformation(self):
        """Calculate transformation matrix from calibration points"""
        try:
            robot_points = np.array(self.robot_points)
            camera_points = np.array(self.camera_points)
            
            # Center the point clouds
            robot_center = np.mean(robot_points, axis=0)
            camera_center = np.mean(camera_points, axis=0)
            
            robot_centered = robot_points - robot_center
            camera_centered = camera_points - camera_center
            
            # Calculate rotation using SVD
            H = robot_centered.T @ camera_centered
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Ensure proper rotation matrix
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # Calculate translation
            t = camera_center - R @ robot_center
            
            # Create 4x4 transformation matrix
            transform = np.eye(4)
            transform[:3, :3] = R
            transform[:3, 3] = t
            
            return transform
            
        except Exception as e:
            self.get_logger().error(f"Error calculating transformation: {e}")
            return None
    
    def _validate_calibration(self):
        """Validate calibration accuracy using test points"""
        if self.transform_matrix is None:
            return float('inf')
        
        errors = []
        robot_points = np.array(self.robot_points)
        camera_points = np.array(self.camera_points)
        
        for i in range(len(robot_points)):
            # Transform robot point to camera frame
            robot_homogeneous = np.append(robot_points[i], 1)
            predicted_camera = self.transform_matrix @ robot_homogeneous
            
            # Calculate error
            error = np.linalg.norm(predicted_camera[:3] - camera_points[i])
            errors.append(error)
        
        return np.mean(errors)
    
    def _publish_calibration_result(self):
        """Publish calibration transformation result"""
        if self.transform_matrix is None:
            return
        
        # Extract rotation and translation
        rotation_matrix = self.transform_matrix[:3, :3]
        translation = self.transform_matrix[:3, 3]
        
        # Convert rotation matrix to quaternion
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # [x, y, z, w]
        
        # Create transform message
        transform_msg = TransformStamped()
        transform_msg.header.stamp = self.get_clock().now().to_msg()
        transform_msg.header.frame_id = self.robot_frame
        transform_msg.child_frame_id = self.camera_frame
        
        transform_msg.transform.translation.x = translation[0] / 1000.0  # Convert mm to m
        transform_msg.transform.translation.y = translation[1] / 1000.0
        transform_msg.transform.translation.z = translation[2] / 1000.0
        
        transform_msg.transform.rotation.x = quaternion[0]
        transform_msg.transform.rotation.y = quaternion[1]
        transform_msg.transform.rotation.z = quaternion[2]
        transform_msg.transform.rotation.w = quaternion[3]
        
        # Publish as static transform
        self.static_tf_broadcaster.sendTransform(transform_msg)
        
        # Also publish as coordinate transform message
        coord_transform = CoordinateTransform()
        coord_transform.source_frame = self.robot_frame
        coord_transform.target_frame = self.camera_frame
        coord_transform.transform = transform_msg.transform
        coord_transform.accuracy = self.calibration_accuracy
        coord_transform.is_valid = self.calibration_accuracy < self.max_error
        coord_transform.timestamp = self.get_clock().now().to_msg()
        
        self.coordinate_transform_pub.publish(coord_transform)
    
    def _save_calibration(self):
        """Save calibration results to file"""
        if self.transform_matrix is None:
            return
        
        # Extract rotation and translation
        rotation_matrix = self.transform_matrix[:3, :3]
        translation = self.transform_matrix[:3, 3]
        
        # Convert to quaternion
        rotation = Rotation.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()  # [x, y, z, w]
        
        # Save to calibration parameters file
        calibration_data = {
            'calibration': {
                'camera': {
                    'translation': translation.tolist(),
                    'rotation': quaternion.tolist()
                },
                'accuracy': float(self.calibration_accuracy),
                'valid': bool(self.calibration_accuracy < self.max_error),
                'timestamp': self.get_clock().now().to_msg().sec
            }
        }
        
        # In a real implementation, save to the actual config file
        self.get_logger().info(f"Calibration data ready for saving: accuracy {self.calibration_accuracy:.2f}mm")


def main(args=None):
    rclpy.init(args=args)
    
    calibration_node = CalibrationNode()
    
    # Start calibration automatically for testing
    # In practice, this would be triggered by user command
    calibration_node.start_calibration()
    
    try:
        rclpy.spin(calibration_node)
    except KeyboardInterrupt:
        pass
    finally:
        calibration_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
