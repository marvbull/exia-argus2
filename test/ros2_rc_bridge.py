#!/usr/bin/env python3
"""
ROS2 RC Bridge Node
Publishes RC channel data as ROS2 topics, subscribes to autonomous control.

Topics:
  Published:
    /rc/channels (exia_msgs/RCChannels) - raw RC data
    /rc/status (std_msgs/String) - "MANUAL", "AUTO", "FAILSAFE"

  Subscribed:
    /cmd_vel (geometry_msgs/Twist) - autonomous velocity commands
    /servo/cmd (std_msgs/Int32) - autonomous servo commands

Usage:
    # Terminal 1: Start ROS2 bridge
    ros2 run exia_argus2 ros2_rc_bridge

    # Terminal 2: Monitor RC data
    ros2 topic echo /rc/channels

    # Terminal 3: Send autonomous commands
    ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 1.0}}'
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32
import serial
import struct
import time
from typing import Optional

# Import your SBUS decoder
# from jetson_to_stm32_bridge_v2 import decode_sbus_frame, sync_to_sbus_frame


class RCBridgeNode(Node):
    def __init__(self):
        super().__init__('rc_bridge')

        # Parameters
        self.declare_parameter('sbus_port', '/dev/sbus')
        self.declare_parameter('stm32_port', '/dev/stm32')
        self.declare_parameter('auto_channel', 5)  # CH6 = auto/manual switch

        # Publishers
        self.rc_pub = self.create_publisher(String, '/rc/channels', 10)
        self.status_pub = self.create_publisher(String, '/rc/status', 10)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.servo_sub = self.create_subscription(
            Int32, '/servo/cmd', self.servo_cmd_callback, 10)

        # State
        self.manual_mode = True
        self.auto_servo = 600  # autonomous servo command
        self.auto_gas = 300    # autonomous gas command
        self.rc_servo = 600
        self.rc_gas = 300

        # Serial setup
        sbus_port = self.get_parameter('sbus_port').value
        stm32_port = self.get_parameter('stm32_port').value

        self.get_logger().info(f'Opening SBUS: {sbus_port}')
        self.sbus = serial.Serial(sbus_port, 100000, parity=serial.PARITY_EVEN, stopbits=2)

        self.get_logger().info(f'Opening STM32: {stm32_port}')
        self.stm32 = serial.Serial(stm32_port, 115200)

        # Timers
        self.create_timer(0.02, self.main_loop)  # 50 Hz main loop
        self.create_timer(0.1, self.publish_status)  # 10 Hz status

    def cmd_vel_callback(self, msg: Twist):
        """Convert ROS cmd_vel to gas value."""
        if not self.manual_mode:
            # Map linear.x (-1 to 1) to gas range (300 to 1200)
            linear_x = max(-1.0, min(1.0, msg.linear.x))
            self.auto_gas = int(300 + (linear_x + 1) * 450)  # 300 to 1200

    def servo_cmd_callback(self, msg: Int32):
        """Direct servo command from autonomous system."""
        if not self.manual_mode:
            self.auto_servo = max(1200, min(1600, msg.data))

    def main_loop(self):
        """Main 50Hz control loop."""
        # Read SBUS (simplified - you'd use your existing decoder)
        try:
            # Read RC channels and check auto/manual switch
            # rc_data = self.read_sbus_frame()
            # if rc_data:
            #     self.rc_servo, self.rc_gas = self.process_rc_channels(rc_data)
            #     auto_switch = rc_data['channels'][5]  # CH6
            #     self.manual_mode = auto_switch < 1500
            pass
        except:
            pass

        # Choose control source
        if self.manual_mode:
            servo_out = self.rc_servo
            gas_out = self.rc_gas
            mode = "MANUAL"
        else:
            servo_out = self.auto_servo
            gas_out = self.auto_gas
            mode = "AUTO"

        # Send to STM32
        try:
            frame = self.build_stm32_frame(servo_out, gas_out, 600, 600)
            self.stm32.write(frame)
        except:
            mode = "FAILSAFE"

        # Publish RC data for other nodes
        msg = String()
        msg.data = f"servo:{servo_out},gas:{gas_out},mode:{mode}"
        self.rc_pub.publish(msg)

    def publish_status(self):
        """Publish system status."""
        status = String()
        status.data = "MANUAL" if self.manual_mode else "AUTO"
        self.status_pub.publish(status)

    def build_stm32_frame(self, ch1: int, ch2: int, ch3: int, ch4: int) -> bytes:
        """Build STM32 frame (copy from your bridge)."""
        payload = struct.pack("<HHHH", ch1, ch2, ch3, ch4)
        # Add sync + CRC logic here
        return b'\\xAA\\x55' + payload + b'\\x00'  # simplified


def main():
    rclpy.init()
    node = RCBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()