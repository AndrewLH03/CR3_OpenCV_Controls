#!/usr/bin/env python3
"""
Safety Monitor Node for CR3 Robot System
Monitors robot position, workspace boundaries, and issues safety alerts.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import math
from enum import IntEnum

from geometry_msgs.msg import Point
from builtin_interfaces.msg import Time
from cr3_hand_control.msg import RobotStatus, SafetyAlert, BasicCommand
from cr3_hand_control.srv import EmergencyStop


class SafetyLevel(IntEnum):
    """Safety alert levels"""
    INFO = 0
    WARNING = 1
    DANGER = 2
    EMERGENCY = 3


class SafetyZone(IntEnum):
    """Workspace safety zones"""
    SAFE = 0
    APPROACH = 1
    WARNING = 2
    DANGER = 3


class SafetyMonitor(Node):
    """
    Main safety monitoring node that tracks robot position and issues safety alerts.
    Monitors workspace boundaries, speeds, and emergency conditions.
    """
    
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Declare parameters
        self._declare_safety_parameters()
        
        # Load safety parameters
        self._load_safety_config()
        
        # Publishers
        self.safety_alert_pub = self.create_publisher(
            SafetyAlert, 
            '/safety_alerts', 
            10
        )
        
        self.safety_command_pub = self.create_publisher(
            BasicCommand,
            '/safety_commands',
            10
        )
        
        # Subscribers
        self.robot_status_sub = self.create_subscription(
            RobotStatus,
            '/robot_status',
            self.robot_status_callback,
            10
        )
        
        # Services
        self.emergency_stop_service = self.create_service(
            EmergencyStop,
            '/emergency_stop',
            self.emergency_stop_callback
        )
        
        # Internal state
        self.current_position = Point()
        self.current_position.x = 0.0  # Start in safe center position
        self.current_position.y = 0.0
        self.current_position.z = 300.0  # Safe height
        self.current_speed = 0.0
        self.last_position = Point()
        self.last_time = self.get_clock().now()
        self.emergency_active = False
        self.last_zone = SafetyZone.SAFE
        self.robot_status_received = False  # Track if we've received any robot data
        
        # Create timer for periodic safety checks
        self.safety_timer = self.create_timer(
            1.0 / self.update_frequency,
            self.safety_check_callback
        )
        
        self.get_logger().info("Safety Monitor initialized and monitoring workspace")
    
    def _declare_safety_parameters(self):
        """Declare all safety-related parameters"""
        # Workspace boundaries
        self.declare_parameter('safety.workspace_boundaries.x_min', -400.0)
        self.declare_parameter('safety.workspace_boundaries.x_max', 400.0)
        self.declare_parameter('safety.workspace_boundaries.y_min', -400.0)
        self.declare_parameter('safety.workspace_boundaries.y_max', 400.0)
        self.declare_parameter('safety.workspace_boundaries.z_min', 50.0)
        self.declare_parameter('safety.workspace_boundaries.z_max', 600.0)
        self.declare_parameter('safety.workspace_boundaries.buffer_zone', 20.0)
        
        # Emergency response
        self.declare_parameter('safety.emergency_response.response_timeout', 0.1)
        
        # Speed limiting
        self.declare_parameter('safety.speed_limiting.minimum_speed', 5.0)
        self.declare_parameter('safety.speed_limiting.safety_speed_factor', 0.5)
        
        # Zones
        self.declare_parameter('safety.zones.danger_zone_distance', 10.0)
        self.declare_parameter('safety.zones.warning_zone_distance', 30.0)
        self.declare_parameter('safety.zones.approach_zone_distance', 50.0)
        
        # Monitoring
        self.declare_parameter('safety.monitoring.update_frequency', 100.0)
    
    def _load_safety_config(self):
        """Load safety configuration from parameters"""
        # Workspace boundaries
        self.x_min = self.get_parameter('safety.workspace_boundaries.x_min').value
        self.x_max = self.get_parameter('safety.workspace_boundaries.x_max').value
        self.y_min = self.get_parameter('safety.workspace_boundaries.y_min').value
        self.y_max = self.get_parameter('safety.workspace_boundaries.y_max').value
        self.z_min = self.get_parameter('safety.workspace_boundaries.z_min').value
        self.z_max = self.get_parameter('safety.workspace_boundaries.z_max').value
        self.buffer_zone = self.get_parameter('safety.workspace_boundaries.buffer_zone').value
        
        # Safety zones
        self.danger_distance = self.get_parameter('safety.zones.danger_zone_distance').value
        self.warning_distance = self.get_parameter('safety.zones.warning_zone_distance').value
        self.approach_distance = self.get_parameter('safety.zones.approach_zone_distance').value
        
        # Update frequency
        self.update_frequency = self.get_parameter('safety.monitoring.update_frequency').value
        
        # Other parameters
        self.response_timeout = self.get_parameter('safety.emergency_response.response_timeout').value
        self.minimum_speed = self.get_parameter('safety.speed_limiting.minimum_speed').value
        self.safety_speed_factor = self.get_parameter('safety.speed_limiting.safety_speed_factor').value
    
    def robot_status_callback(self, msg):
        """Process incoming robot status updates"""
        self.robot_status_received = True
        
        # Store previous position for speed calculation
        self.last_position = Point(
            x=self.current_position.x,
            y=self.current_position.y,
            z=self.current_position.z
        )
        
        # Update current position from robot status
        if len(msg.current_position) >= 3:
            self.current_position.x = msg.current_position[0]
            self.current_position.y = msg.current_position[1]
            self.current_position.z = msg.current_position[2]
        
        # Calculate speed
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            dx = self.current_position.x - self.last_position.x
            dy = self.current_position.y - self.last_position.y
            dz = self.current_position.z - self.last_position.z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            self.current_speed = distance / dt
        
        self.last_time = current_time
    
    def safety_check_callback(self):
        """Periodic safety checks"""
        if self.emergency_active:
            return
        
        # Only perform checks if we've received robot status data
        if not self.robot_status_received:
            return
            
        # Check workspace boundaries
        self._check_workspace_boundaries()
        
        # Check speed limits
        self._check_speed_limits()
    
    def _check_workspace_boundaries(self):
        """Check if robot is within safe workspace boundaries"""
        pos = self.current_position
        
        # Calculate distance to each boundary
        distances = {
            'x_min': pos.x - self.x_min,
            'x_max': self.x_max - pos.x,
            'y_min': pos.y - self.y_min,
            'y_max': self.y_max - pos.y,
            'z_min': pos.z - self.z_min,
            'z_max': self.z_max - pos.z
        }
        
        # Find minimum distance to any boundary
        min_distance = min(distances.values())
        boundary_name = min(distances, key=distances.get)
        
        # Determine safety zone
        current_zone = self._get_safety_zone(min_distance)
        
        # Issue alerts based on zone
        if current_zone != self.last_zone:
            self._issue_boundary_alert(current_zone, min_distance, boundary_name)
            self.last_zone = current_zone
        
        # Take action if in danger zone
        if current_zone >= SafetyZone.DANGER:
            self._initiate_emergency_stop("Boundary violation imminent")
    
    def _get_safety_zone(self, distance_to_boundary):
        """Determine safety zone based on distance to boundary"""
        if distance_to_boundary <= self.danger_distance:
            return SafetyZone.DANGER
        elif distance_to_boundary <= self.warning_distance:
            return SafetyZone.WARNING
        elif distance_to_boundary <= self.approach_distance:
            return SafetyZone.APPROACH
        else:
            return SafetyZone.SAFE
    
    def _issue_boundary_alert(self, zone, distance, boundary):
        """Issue safety alert for boundary approach"""
        alert = SafetyAlert()
        alert.location = self.current_position
        alert.distance_to_boundary = distance
        alert.timestamp = self.get_clock().now().to_msg()
        alert.alert_type = "boundary"
        
        if zone == SafetyZone.DANGER:
            alert.level = SafetyLevel.DANGER
            alert.description = f"DANGER: Within {distance:.1f}mm of {boundary} boundary"
            alert.recommended_speed = 0.0
        elif zone == SafetyZone.WARNING:
            alert.level = SafetyLevel.WARNING
            alert.description = f"WARNING: Within {distance:.1f}mm of {boundary} boundary"
            alert.recommended_speed = self.current_speed * self.safety_speed_factor
        elif zone == SafetyZone.APPROACH:
            alert.level = SafetyLevel.INFO
            alert.description = f"INFO: Approaching {boundary} boundary, {distance:.1f}mm remaining"
            alert.recommended_speed = self.current_speed * 0.8
        else:
            alert.level = SafetyLevel.INFO
            alert.description = "Safe zone - normal operation"
            alert.recommended_speed = -1.0  # No limit
        
        self.safety_alert_pub.publish(alert)
        self.get_logger().info(f"Safety Alert: {alert.description}")
    
    def _check_speed_limits(self):
        """Check if current speed is within safe limits"""
        # Get recommended speed based on distance to boundary
        pos = self.current_position
        min_distance = self._get_min_boundary_distance(pos)
        
        if min_distance <= self.approach_distance:
            # Calculate safe speed based on distance
            speed_factor = min_distance / self.approach_distance
            safe_speed = max(self.minimum_speed, self.current_speed * speed_factor)
            
            if self.current_speed > safe_speed:
                self._issue_speed_alert(safe_speed)
    
    def _get_min_boundary_distance(self, pos):
        """Get minimum distance to any workspace boundary"""
        distances = [
            pos.x - self.x_min,
            self.x_max - pos.x,
            pos.y - self.y_min,
            self.y_max - pos.y,
            pos.z - self.z_min,
            self.z_max - pos.z
        ]
        return min(distances)
    
    def _issue_speed_alert(self, recommended_speed):
        """Issue speed limit alert"""
        alert = SafetyAlert()
        alert.level = SafetyLevel.WARNING
        alert.alert_type = "speed"
        alert.description = f"Speed limit: reduce to {recommended_speed:.1f}mm/s"
        alert.location = self.current_position
        alert.recommended_speed = recommended_speed
        alert.timestamp = self.get_clock().now().to_msg()
        
        self.safety_alert_pub.publish(alert)
    
    def _initiate_emergency_stop(self, reason):
        """Initiate emergency stop sequence"""
        if self.emergency_active:
            return
        
        self.emergency_active = True
        
        # Send emergency stop command
        stop_command = BasicCommand()
        stop_command.command_type = "Stop"
        stop_command.position = [self.current_position.x, self.current_position.y, self.current_position.z, 0.0, 0.0, 0.0]
        stop_command.velocity = 0.0
        stop_command.relative = False
        
        self.safety_command_pub.publish(stop_command)
        
        # Issue emergency alert
        alert = SafetyAlert()
        alert.level = SafetyLevel.EMERGENCY
        alert.alert_type = "system"
        alert.description = f"EMERGENCY STOP: {reason}"
        alert.location = self.current_position
        alert.recommended_speed = 0.0
        alert.timestamp = self.get_clock().now().to_msg()
        
        self.safety_alert_pub.publish(alert)
        
        self.get_logger().error(f"EMERGENCY STOP ACTIVATED: {reason}")
    
    def emergency_stop_callback(self, request, response):
        """Handle emergency stop service requests"""
        if request.stop_type == "emergency":
            self._initiate_emergency_stop("Manual emergency stop")
            response.success = True
            response.message = "Emergency stop activated"
        elif request.stop_type == "reset":
            self.emergency_active = False
            response.success = True
            response.message = "Emergency stop reset"
            self.get_logger().info("Emergency stop reset - resuming normal operation")
        else:
            response.success = False
            response.message = f"Unknown stop type: {request.stop_type}"
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    safety_monitor = SafetyMonitor()
    
    try:
        rclpy.spin(safety_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        safety_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
