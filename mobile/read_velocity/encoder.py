#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import re

class STM32Receiver(Node):
    def __init__(self):
        super().__init__("stm32_receiver")

        # ROS publisher
        self.pub = self.create_publisher(Twist, "/wheel_speed", 10)

        # Open Serial port
        try:
            self.ser = serial.Serial('/dev/cp210x_stm', 115200, timeout=0.1)
            self.get_logger().info("Connected to /dev/STM")
        except Exception as e:
            self.get_logger().error(f"Serial open error: {e}")
            exit(1)

        # Timer 50 Hz
        self.timer = self.create_timer(0.02, self.loop)

        # constants
        self.wheel_base = 0.42

    def loop(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except:
            return

        if not line.startswith("vL"):
            return  

        # Expected format:
        # vL = 0.198 m/s   vR = 0.195 m/s
        nums = re.findall(r"[-+]?\d*\.\d+|\d+", line)

        if len(nums) < 2:
            return
        
        vL = float(nums[0])
        vR = float(nums[1])

        # robot velocity
        vx = 0.5 * (vL + vR)
        wz = (vR - vL) / self.wheel_base

        # Publish Twist
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        self.pub.publish(msg)

        self.get_logger().info(f"vx={vx:.3f} m/s   wz={wz:.3f} rad/s")


def main(args=None):
    rclpy.init(args=args)
    node = STM32Receiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 