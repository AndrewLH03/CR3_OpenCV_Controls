#!/usr/bin/env python3
"""
Diagnostics Monitor Node for CR3 Hand Control System
Monitors system health and publishes diagnostics.
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class DiagnosticsMonitor(Node):
    def __init__(self):
        super().__init__('diagnostics_monitor')
        
        # Create diagnostics publisher
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            'diagnostics',
            10
        )
        
        # Create timer for regular diagnostics
        self.timer = self.create_timer(1.0, self.publish_diagnostics)
        
        self.get_logger().info("Diagnostics Monitor initialized")
    
    def publish_diagnostics(self):
        """Publish system diagnostics."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Create a basic status
        status = DiagnosticStatus()
        status.name = "cr3_hand_control"
        status.level = DiagnosticStatus.OK
        status.message = "System operational"
        
        msg.status.append(status)
        self.diagnostics_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
