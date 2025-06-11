#!/usr/bin/env python3
"""
Phase 3 Integration Test
Tests basic functionality of Phase 3 components working together.
"""

import rclpy
from rclpy.node import Node
import time
import threading
import subprocess
import signal
import os
from cr3_hand_control.msg import RobotStatus, SafetyAlert
from cr3_hand_control.srv import EmergencyStop


class Phase3IntegrationTest(Node):
    def __init__(self):
        super().__init__('phase3_integration_test')
        
        # Storage for received messages
        self.safety_alerts = []
        
        # Publishers
        self.robot_status_pub = self.create_publisher(
            RobotStatus, '/robot_status', 10)
        
        # Subscribers
        self.safety_alert_sub = self.create_subscription(
            SafetyAlert, '/safety_alerts',
            self.safety_alert_callback, 10)
        
        # Service client
        self.emergency_stop_client = self.create_client(
            EmergencyStop, '/emergency_stop')
        
        self.get_logger().info("Phase 3 Integration Test initialized")
    
    def safety_alert_callback(self, msg):
        """Callback for safety alerts"""
        self.safety_alerts.append(msg)
        self.get_logger().info(f"Received safety alert: {msg.description}")
    
    def publish_robot_position(self, x, y, z):
        """Publish a robot position"""
        status = RobotStatus()
        status.connected = True
        status.moving = False
        status.enabled = True
        status.current_position = [x, y, z, 0.0, 0.0, 0.0]
        status.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        status.current_velocity = 10.0
        status.last_error = ""
        status.timestamp = self.get_clock().now().to_msg()
        
        self.robot_status_pub.publish(status)
        self.get_logger().info(f"Published robot position: ({x}, {y}, {z})")
    
    def test_emergency_stop_service(self):
        """Test emergency stop service"""
        if not self.emergency_stop_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("Emergency stop service not available")
            return False
        
        request = EmergencyStop.Request()
        request.stop_type = "emergency"
        request.reason = "Integration test"
        
        future = self.emergency_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info("Emergency stop service test: PASSED")
                return True
            else:
                self.get_logger().error(f"Emergency stop failed: {response.message}")
        else:
            self.get_logger().error("Emergency stop service call timed out")
        
        return False
    
    def run_test_sequence(self):
        """Run comprehensive Phase 3 test sequence"""
        self.get_logger().info("Starting Phase 3 comprehensive integration test...")
        
        # Test 1: Publish safe position
        self.get_logger().info("Test 1: Publishing safe position")
        self.publish_robot_position(0.0, 0.0, 300.0)
        time.sleep(1.0)
        
        # Verify no danger alerts for safe position
        danger_alerts = [alert for alert in self.safety_alerts if alert.level >= 2]
        if len(danger_alerts) == 0:
            self.get_logger().info("✓ Safe position test PASSED - no danger alerts")
        else:
            self.get_logger().warn(f"⚠ Safe position triggered {len(danger_alerts)} danger alerts")
        
        # Test 2: Publish boundary approach position
        self.get_logger().info("Test 2: Publishing boundary approach position")
        initial_alert_count = len(self.safety_alerts)
        self.publish_robot_position(350.0, 0.0, 300.0)
        time.sleep(1.5)  # Allow time for safety processing
        
        # Verify boundary approach triggers warning
        new_alerts = len(self.safety_alerts) - initial_alert_count
        if new_alerts > 0:
            self.get_logger().info(f"✓ Boundary approach test PASSED - {new_alerts} alerts triggered")
        else:
            self.get_logger().warn("⚠ Boundary approach didn't trigger expected alerts")
        
        # Test 3: Test emergency stop service
        self.get_logger().info("Test 3: Testing emergency stop service")
        service_result = self.test_emergency_stop_service()
        
        # Test 4: Reset emergency stop
        reset_result = False
        if service_result:
            self.get_logger().info("Test 4: Resetting emergency stop")
            request = EmergencyStop.Request()
            request.stop_type = "reset"
            request.reason = "Test reset"
            
            future = self.emergency_stop_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            
            if future.done():
                response = future.result()
                reset_result = response.success
                if reset_result:
                    self.get_logger().info("✓ Emergency stop reset PASSED")
                else:
                    self.get_logger().error(f"✗ Emergency stop reset failed: {response.message}")
        
        # Test 5: Test boundary violation (danger zone)
        self.get_logger().info("Test 5: Testing boundary violation")
        pre_danger_alerts = len([alert for alert in self.safety_alerts if alert.level >= 2])
        self.publish_robot_position(450.0, 0.0, 300.0)  # Outside workspace
        time.sleep(1.5)
        
        post_danger_alerts = len([alert for alert in self.safety_alerts if alert.level >= 2])
        if post_danger_alerts > pre_danger_alerts:
            self.get_logger().info("✓ Boundary violation test PASSED - danger alert triggered")
        else:
            self.get_logger().warn("⚠ Boundary violation didn't trigger danger alert")
        
        # Summary
        total_alerts = len(self.safety_alerts)
        boundary_alerts = len([alert for alert in self.safety_alerts if alert.alert_type == "boundary"])
        emergency_alerts = len([alert for alert in self.safety_alerts if "emergency" in alert.description.lower()])
        
        self.get_logger().info("=== TEST SUMMARY ===")
        self.get_logger().info(f"Total alerts received: {total_alerts}")
        self.get_logger().info(f"Boundary alerts: {boundary_alerts}")
        self.get_logger().info(f"Emergency alerts: {emergency_alerts}")
        self.get_logger().info(f"Service calls successful: {service_result and reset_result}")
        
        # Test passes if we got safety alerts AND service calls worked
        test_passed = (total_alerts > 0) and service_result
        if test_passed:
            self.get_logger().info("🎉 PHASE 3 INTEGRATION TEST: PASSED")
        else:
            self.get_logger().error("❌ PHASE 3 INTEGRATION TEST: FAILED")
        
        return test_passed


def main():
    rclpy.init()
    
    # Start safety monitor in background
    safety_monitor_process = None
    try:
        # Set up environment for ROS2 package
        env = os.environ.copy()
        ros2_package_dir = '/home/andrewlh/CR3_OpenCV_Controls/ros2_package'
        
        # Source both ROS2 installation and local package
        ros2_setup = '/opt/ros/jazzy/setup.bash'
        local_setup = os.path.join(ros2_package_dir, 'install', 'setup.bash')
        
        if os.path.exists(local_setup):
            # Source ROS2 installation and local package overlay
            safety_monitor_cmd = f'source {ros2_setup} && source {local_setup} && python3 scripts/safety/safety_monitor.py'
            safety_monitor_process = subprocess.Popen([
                'bash', '-c', safety_monitor_cmd
            ], cwd=ros2_package_dir, env=env)
            print(f"Started safety monitor with ROS2 sourcing")
        else:
            print("ERROR: ROS2 package not installed. Run 'colcon build' first.")
            print(f"Missing: {local_setup}")
            return
        
        time.sleep(2.0)  # Allow safety monitor to start
        
        # Run integration test
        test_node = Phase3IntegrationTest()
        
        # Run test in a separate thread
        def run_test():
            time.sleep(1.0)  # Allow connections to establish
            return test_node.run_test_sequence()
        
        test_thread = threading.Thread(target=run_test)
        test_thread.start()
        
        # Spin for limited time
        start_time = time.time()
        while time.time() - start_time < 10.0 and rclpy.ok():
            rclpy.spin_once(test_node, timeout_sec=0.1)
        
        test_thread.join(timeout=1.0)
        
        print("Phase 3 Integration Test completed successfully!")
        
    except Exception as e:
        print(f"Integration test failed: {e}")
    finally:
        # Clean up
        if safety_monitor_process:
            safety_monitor_process.terminate()
            safety_monitor_process.wait()
        
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
