#!/usr/bin/env python3
"""
Parameter Manager Node

Provides runtime parameter management with validation and QoS configuration
for the CR3 Hand Control System.

Features:
- Runtime parameter validation and updates
- QoS profile management
- Parameter change callbacks
- Configuration file reloading
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from cr3_hand_control.srv import SetParameters
import yaml
import os
from typing import Dict, Any, Optional


class ParameterManager(Node):
    """
    Advanced parameter management system for CR3 Hand Control.
    
    Handles runtime parameter updates, validation, and QoS profile configuration.
    """
    
    def __init__(self):
        super().__init__('parameter_manager')
        
        # Load initial parameters from config file
        self._load_config_file()
        
        # Setup parameter descriptors with validation
        self._setup_parameter_descriptors()
        
        # Create services
        self._setup_services()
        
        # Configure QoS profiles
        self._configure_qos_profiles()
        
        self.get_logger().info("Parameter Manager initialized successfully")
    
    def _load_config_file(self):
        """Load parameters from YAML configuration file."""
        try:
            config_path = os.path.join(
                self.get_package_share_directory('cr3_hand_control'),
                'config', 'robot_params.yaml'
            )
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    config = yaml.safe_load(file)
                    if '/**' in config and 'ros__parameters' in config['/**']:
                        params = config['/**']['ros__parameters']
                        self._apply_loaded_parameters(params)
            else:
                self.get_logger().warn(f"Config file not found: {config_path}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to load config file: {e}")
    
    def _apply_loaded_parameters(self, params: Dict[str, Any], prefix: str = ""):
        """Recursively apply loaded parameters."""
        for key, value in params.items():
            param_name = f"{prefix}.{key}" if prefix else key
            
            if isinstance(value, dict):
                self._apply_loaded_parameters(value, param_name)
            else:
                try:
                    self.declare_parameter(param_name, value)
                except Exception as e:
                    self.get_logger().warn(f"Failed to declare parameter {param_name}: {e}")
    
    def _setup_parameter_descriptors(self):
        """Setup parameter descriptors with validation rules."""
        descriptors = {
            'robot.ip_address': ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Robot IP address for TCP connection"
            ),
            'robot.control_port': ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Robot control port (typically 29999)"
            ),
            'movement.max_velocity': ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum robot velocity in m/s"
            ),
            'safety.emergency_stop_timeout': ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Emergency stop response timeout in seconds"
            ),
            'hand_tracking.detection_confidence': ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Hand detection confidence threshold (0.0-1.0)"
            )
        }
        
        for param_name, descriptor in descriptors.items():
            try:
                if not self.has_parameter(param_name):
                    self.declare_parameter(param_name, descriptor=descriptor)
            except Exception as e:
                self.get_logger().warn(f"Failed to setup descriptor for {param_name}: {e}")
    
    def _setup_services(self):
        """Setup parameter management services."""
        self.set_parameters_service = self.create_service(
            SetParameters,
            'set_parameters',
            self._handle_set_parameters
        )
        
        self.get_logger().info("Parameter management services created")
    
    def _configure_qos_profiles(self):
        """Configure QoS profiles based on parameters."""
        try:
            # Robot commands QoS
            robot_qos = self._create_qos_profile('qos.robot_commands')
            
            # Hand tracking QoS  
            tracking_qos = self._create_qos_profile('qos.hand_tracking')
            
            # Safety status QoS
            safety_qos = self._create_qos_profile('qos.safety_status')
            
            # Store QoS profiles for use by other nodes
            self._qos_profiles = {
                'robot_commands': robot_qos,
                'hand_tracking': tracking_qos,
                'safety_status': safety_qos
            }
            
            self.get_logger().info("QoS profiles configured successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to configure QoS profiles: {e}")
    
    def _create_qos_profile(self, param_prefix: str) -> QoSProfile:
        """Create QoS profile from parameters."""
        try:
            reliability_str = self.get_parameter(f'{param_prefix}.reliability').value
            durability_str = self.get_parameter(f'{param_prefix}.durability').value
            deadline_ms = self.get_parameter(f'{param_prefix}.deadline_ms').value
            
            # Map string values to enums
            reliability_map = {
                'reliable': ReliabilityPolicy.RELIABLE,
                'best_effort': ReliabilityPolicy.BEST_EFFORT
            }
            
            durability_map = {
                'volatile': DurabilityPolicy.VOLATILE,
                'transient_local': DurabilityPolicy.TRANSIENT_LOCAL
            }
            
            qos = QoSProfile(depth=10)
            qos.reliability = reliability_map.get(reliability_str, ReliabilityPolicy.RELIABLE)
            qos.durability = durability_map.get(durability_str, DurabilityPolicy.VOLATILE)
            
            return qos
            
        except Exception as e:
            self.get_logger().warn(f"Failed to create QoS profile for {param_prefix}: {e}")
            return QoSProfile(depth=10)  # Default profile
    
    def _handle_set_parameters(self, request, response):
        """Handle parameter setting service requests."""
        try:
            param_name = request.parameter_name
            param_value = request.parameter_value
            validate_only = request.validate_only
            
            # Get current value
            if self.has_parameter(param_name):
                old_param = self.get_parameter(param_name)
                old_value = str(old_param.value)
            else:
                old_value = "undefined"
            
            # Validate parameter
            validation_result = self._validate_parameter(param_name, param_value)
            
            if not validation_result['valid']:
                response.success = False
                response.message = validation_result['error']
                response.old_value = old_value
                response.requires_restart = False
                return response
            
            # If only validation was requested, return success
            if validate_only:
                response.success = True
                response.message = "Parameter validation successful"
                response.old_value = old_value
                response.requires_restart = validation_result.get('requires_restart', False)
                return response
            
            # Set the parameter
            try:
                converted_value = self._convert_parameter_value(param_name, param_value)
                self.set_parameters([Parameter(param_name, value=converted_value)])
                
                response.success = True
                response.message = f"Parameter {param_name} updated successfully"
                response.old_value = old_value
                response.requires_restart = validation_result.get('requires_restart', False)
                
                self.get_logger().info(f"Parameter updated: {param_name} = {param_value}")
                
            except Exception as e:
                response.success = False
                response.message = f"Failed to set parameter: {e}"
                response.old_value = old_value
                response.requires_restart = False
                
        except Exception as e:
            response.success = False
            response.message = f"Service error: {e}"
            response.old_value = "error"
            response.requires_restart = False
        
        return response
    
    def _validate_parameter(self, param_name: str, param_value: str) -> Dict[str, Any]:
        """Validate parameter value and return validation result."""
        try:
            # Define validation rules
            validation_rules = {
                'robot.ip_address': self._validate_ip_address,
                'robot.control_port': lambda v: self._validate_port(v),
                'movement.max_velocity': lambda v: self._validate_range(v, 0.1, 2.0),
                'movement.max_acceleration': lambda v: self._validate_range(v, 0.1, 5.0),
                'hand_tracking.detection_confidence': lambda v: self._validate_range(v, 0.0, 1.0),
                'safety.emergency_stop_timeout': lambda v: self._validate_range(v, 0.01, 1.0),
            }
            
            if param_name in validation_rules:
                return validation_rules[param_name](param_value)
            else:
                # Default validation - just check if convertible
                self._convert_parameter_value(param_name, param_value)
                return {'valid': True, 'requires_restart': False}
                
        except Exception as e:
            return {'valid': False, 'error': f"Validation error: {e}"}
    
    def _validate_ip_address(self, ip_str: str) -> Dict[str, Any]:
        """Validate IP address format."""
        import ipaddress
        try:
            ipaddress.ip_address(ip_str)
            return {'valid': True, 'requires_restart': True}
        except ValueError:
            return {'valid': False, 'error': "Invalid IP address format"}
    
    def _validate_port(self, port_str: str) -> Dict[str, Any]:
        """Validate port number."""
        try:
            port = int(port_str)
            if 1 <= port <= 65535:
                return {'valid': True, 'requires_restart': True}
            else:
                return {'valid': False, 'error': "Port must be between 1 and 65535"}
        except ValueError:
            return {'valid': False, 'error': "Port must be an integer"}
    
    def _validate_range(self, value_str: str, min_val: float, max_val: float) -> Dict[str, Any]:
        """Validate numeric value within range."""
        try:
            value = float(value_str)
            if min_val <= value <= max_val:
                return {'valid': True, 'requires_restart': False}
            else:
                return {'valid': False, 'error': f"Value must be between {min_val} and {max_val}"}
        except ValueError:
            return {'valid': False, 'error': "Value must be a number"}
    
    def _convert_parameter_value(self, param_name: str, param_value: str):
        """Convert parameter value to appropriate type."""
        if self.has_parameter(param_name):
            current_param = self.get_parameter(param_name)
            param_type = current_param.type_
            
            if param_type == Parameter.Type.BOOL:
                return param_value.lower() in ('true', '1', 'yes', 'on')
            elif param_type == Parameter.Type.INTEGER:
                return int(param_value)
            elif param_type == Parameter.Type.DOUBLE:
                return float(param_value)
            elif param_type == Parameter.Type.STRING:
                return param_value
            else:
                return param_value
        else:
            # Try to infer type
            try:
                return int(param_value)
            except ValueError:
                try:
                    return float(param_value)
                except ValueError:
                    return param_value
    
    def get_qos_profile(self, profile_name: str) -> Optional[QoSProfile]:
        """Get QoS profile by name."""
        return self._qos_profiles.get(profile_name)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        parameter_manager = ParameterManager()
        rclpy.spin(parameter_manager)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in parameter manager: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
