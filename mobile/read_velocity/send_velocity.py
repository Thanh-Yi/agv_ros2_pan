#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

class TeleopToSTM(Node):
    def __init__(self):
        super().__init__("teleop_to_stm")

        # mở UART
        self.ser = serial.Serial('/dev/cp210x_stm', 115200)

        # sub cmd_vel
        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info("Teleop → STM32 started.")

    def cmd_callback(self, msg: Twist):
        lx = msg.linear.x
        az = msg.angular.z

        # đóng gói frame
        frame = bytearray(10)
        frame[0] = 0xAA
        frame[9] = 0x55

        # pack float -> 4 bytes little-endian giống STM32
        frame[1:5] = struct.pack('<f', lx)
        frame[5:9] = struct.pack('<f', az)

        # gửi xuống STM32
        self.ser.write(frame)

        self.get_logger().info(f"Sent: lx={lx:.3f}  az={az:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopToSTM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
