#!/usr/bin/env python3
"""
Diagnostics Monitor Node

Provides comprehensive system diagnostics and performance monitoring
for the CR3 Hand Control System.

Features:
- Real-time system health monitoring
- Performance metrics collection
- Diagnostic status publishing
- Automated issue detection and alerting
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_msgs.msg import String
import psutil
import time
import threading
from typing import Dict, List, Any
import json


class DiagnosticsMonitor(Node):
    """
    System diagnostics and performance monitoring for CR3 Hand Control.
    
    Monitors system health, performance metrics, and publishes diagnostic status.
    """
    
    def __init__(self):
        super().__init__('diagnostics_monitor')
        
        # Load parameters
        self._load_parameters()
        
        # Initialize diagnostics data
        self._diagnostics_data = {}
        self._performance_metrics = {}
        
        # Setup publishers
        self._setup_publishers()
        
        # Start monitoring threads
        self._start_monitoring()
        
        self.get_logger().info("Diagnostics Monitor initialized successfully")
    
    def _load_parameters(self):
        """Load monitoring parameters."""
        self.declare_parameter('performance.monitor_enabled', True)
        self.declare_parameter('performance.stats_publish_rate', 1.0)
        self.declare_parameter('performance.latency_threshold_ms', 50)
        self.declare_parameter('performance.cpu_threshold_percent', 80)
        self.declare_parameter('logging.level', 'INFO')
        
        self.monitor_enabled = self.get_parameter('performance.monitor_enabled').value
        self.publish_rate = self.get_parameter('performance.stats_publish_rate').value
        self.latency_threshold = self.get_parameter('performance.latency_threshold_ms').value
        self.cpu_threshold = self.get_parameter('performance.cpu_threshold_percent').value
        
    def _setup_publishers(self):
        """Setup diagnostic publishers."""
        # Diagnostic status publisher
        diagnostic_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            diagnostic_qos
        )
        
        # Performance metrics publisher
        self.performance_pub = self.create_publisher(
            String,
            'performance_metrics',
            10
        )
        
        # Setup diagnostic timer
        self.diagnostic_timer = self.create_timer(
            1.0 / self.publish_rate,
            self._publish_diagnostics
        )
    
    def _start_monitoring(self):
        """Start background monitoring threads."""
        if self.monitor_enabled:
            self._monitoring_active = True
            
            # System monitoring thread
            self._system_thread = threading.Thread(
                target=self._monitor_system_health,
                daemon=True
            )
            self._system_thread.start()
            
            # Performance monitoring thread
            self._performance_thread = threading.Thread(
                target=self._monitor_performance,
                daemon=True
            )
            self._performance_thread.start()
    
    def _monitor_system_health(self):
        """Monitor system health metrics."""
        while self._monitoring_active and rclpy.ok():
            try:
                # CPU usage
                cpu_percent = psutil.cpu_percent(interval=1.0)
                
                # Memory usage
                memory = psutil.virtual_memory()
                memory_percent = memory.percent
                
                # Disk usage
                disk = psutil.disk_usage('/')
                disk_percent = disk.percent
                
                # Network statistics
                network = psutil.net_io_counters()
                
                # Update diagnostics data
                self._diagnostics_data.update({
                    'system': {
                        'cpu_percent': cpu_percent,
                        'memory_percent': memory_percent,
                        'disk_percent': disk_percent,
                        'network_bytes_sent': network.bytes_sent,
                        'network_bytes_recv': network.bytes_recv,
                        'timestamp': time.time()
                    }
                })
                
            except Exception as e:
                self.get_logger().error(f"System monitoring error: {e}")
                time.sleep(1.0)
    
    def _monitor_performance(self):
        """Monitor performance metrics."""
        while self._monitoring_active and rclpy.ok():
            try:
                # Process-specific metrics
                process = psutil.Process()
                
                # CPU and memory for this process
                process_cpu = process.cpu_percent()
                process_memory = process.memory_info()
                
                # ROS-specific metrics (simplified)
                ros_metrics = self._collect_ros_metrics()
                
                # Update performance metrics
                self._performance_metrics.update({
                    'process': {
                        'cpu_percent': process_cpu,
                        'memory_rss_mb': process_memory.rss / 1024 / 1024,
                        'memory_vms_mb': process_memory.vms / 1024 / 1024,
                        'timestamp': time.time()
                    },
                    'ros': ros_metrics
                })
                
                time.sleep(1.0)
                
            except Exception as e:
                self.get_logger().error(f"Performance monitoring error: {e}")
                time.sleep(1.0)
    
    def _collect_ros_metrics(self) -> Dict[str, Any]:
        """Collect ROS-specific performance metrics."""
        try:
            # Get node information
            node_names = self.get_node_names()
            topic_names = self.get_topic_names_and_types()
            
            return {
                'active_nodes': len(node_names),
                'active_topics': len(topic_names),
                'node_count': len([name for name in node_names if 'cr3_hand_control' in name]),
                'timestamp': time.time()
            }
        except Exception as e:
            self.get_logger().warn(f"Failed to collect ROS metrics: {e}")
            return {'timestamp': time.time()}
    
    def _publish_diagnostics(self):
        """Publish diagnostic status."""
        try:
            diagnostic_array = DiagnosticArray()
            diagnostic_array.header.stamp = self.get_clock().now().to_msg()
            
            # System diagnostics
            system_status = self._create_system_diagnostic()
            diagnostic_array.status.append(system_status)
            
            # Performance diagnostics
            performance_status = self._create_performance_diagnostic()
            diagnostic_array.status.append(performance_status)
            
            # ROS diagnostics
            ros_status = self._create_ros_diagnostic()
            diagnostic_array.status.append(ros_status)
            
            # Publish diagnostics
            self.diagnostics_pub.publish(diagnostic_array)
            
            # Publish performance metrics as JSON
            if self._performance_metrics:
                performance_json = json.dumps(self._performance_metrics, indent=2)
                performance_msg = String()
                performance_msg.data = performance_json
                self.performance_pub.publish(performance_msg)
                
        except Exception as e:
            self.get_logger().error(f"Failed to publish diagnostics: {e}")
    
    def _create_system_diagnostic(self) -> DiagnosticStatus:
        """Create system diagnostic status."""
        status = DiagnosticStatus()
        status.name = "CR3 Hand Control System Health"
        status.hardware_id = "system"
        
        if 'system' in self._diagnostics_data:
            system_data = self._diagnostics_data['system']
            cpu_percent = system_data.get('cpu_percent', 0)
            memory_percent = system_data.get('memory_percent', 0)
            disk_percent = system_data.get('disk_percent', 0)
            
            # Determine status level
            if cpu_percent > self.cpu_threshold or memory_percent > 90 or disk_percent > 90:
                status.level = DiagnosticStatus.ERROR
                status.message = "System resources critically high"
            elif cpu_percent > 60 or memory_percent > 75 or disk_percent > 75:
                status.level = DiagnosticStatus.WARN
                status.message = "System resources elevated"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "System resources normal"
            
            # Add key-value pairs
            status.values = [
                KeyValue(key="CPU Usage %", value=f"{cpu_percent:.1f}"),
                KeyValue(key="Memory Usage %", value=f"{memory_percent:.1f}"),
                KeyValue(key="Disk Usage %", value=f"{disk_percent:.1f}"),
                KeyValue(key="Network Sent MB", value=f"{system_data.get('network_bytes_sent', 0) / 1024 / 1024:.1f}"),
                KeyValue(key="Network Recv MB", value=f"{system_data.get('network_bytes_recv', 0) / 1024 / 1024:.1f}")
            ]
        else:
            status.level = DiagnosticStatus.STALE
            status.message = "No system data available"
        
        return status
    
    def _create_performance_diagnostic(self) -> DiagnosticStatus:
        """Create performance diagnostic status."""
        status = DiagnosticStatus()
        status.name = "CR3 Hand Control Performance"
        status.hardware_id = "performance"
        
        if 'process' in self._performance_metrics:
            process_data = self._performance_metrics['process']
            process_cpu = process_data.get('cpu_percent', 0)
            process_memory = process_data.get('memory_rss_mb', 0)
            
            # Determine status level
            if process_cpu > 50 or process_memory > 500:
                status.level = DiagnosticStatus.WARN
                status.message = "High resource usage by process"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "Process performance normal"
            
            # Add key-value pairs
            status.values = [
                KeyValue(key="Process CPU %", value=f"{process_cpu:.1f}"),
                KeyValue(key="Process Memory MB", value=f"{process_memory:.1f}"),
                KeyValue(key="Virtual Memory MB", value=f"{process_data.get('memory_vms_mb', 0):.1f}")
            ]
        else:
            status.level = DiagnosticStatus.STALE
            status.message = "No performance data available"
        
        return status
    
    def _create_ros_diagnostic(self) -> DiagnosticStatus:
        """Create ROS diagnostic status."""
        status = DiagnosticStatus()
        status.name = "CR3 Hand Control ROS Status"
        status.hardware_id = "ros"
        
        if 'ros' in self._performance_metrics:
            ros_data = self._performance_metrics['ros']
            active_nodes = ros_data.get('active_nodes', 0)
            active_topics = ros_data.get('active_topics', 0)
            cr3_nodes = ros_data.get('node_count', 0)
            
            # Determine status level
            if cr3_nodes == 0:
                status.level = DiagnosticStatus.ERROR
                status.message = "No CR3 nodes active"
            elif active_nodes < 3:
                status.level = DiagnosticStatus.WARN
                status.message = "Few ROS nodes active"
            else:
                status.level = DiagnosticStatus.OK
                status.message = "ROS system operating normally"
            
            # Add key-value pairs
            status.values = [
                KeyValue(key="Active Nodes", value=str(active_nodes)),
                KeyValue(key="Active Topics", value=str(active_topics)),
                KeyValue(key="CR3 Nodes", value=str(cr3_nodes))
            ]
        else:
            status.level = DiagnosticStatus.STALE
            status.message = "No ROS data available"
        
        return status
    
    def add_custom_diagnostic(self, name: str, level: int, message: str, values: List[KeyValue] = None):
        """Add custom diagnostic status."""
        # This method can be called by other nodes to add custom diagnostics
        # Implementation would store custom diagnostics for inclusion in published array
        pass
    
    def __del__(self):
        """Cleanup monitoring threads."""
        if hasattr(self, '_monitoring_active'):
            self._monitoring_active = False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        diagnostics_monitor = DiagnosticsMonitor()
        rclpy.spin(diagnostics_monitor)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in diagnostics monitor: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
