#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import serial
import re


class STM32Receiver(Node):
    def __init__(self):
        super().__init__("stm32_receiver")

        # ===== Publishers =====
        self.pub_odom = self.create_publisher(
            Odometry, "/wheel_odom", 10
        )

        # (optional) debug
        self.pub_twist = self.create_publisher(
            Twist, "/wheel_speed", 10
        )

        # ===== Serial =====
        try:
            self.ser = serial.Serial(
                "/dev/cp210x_stm", 115200, timeout=0.1
            )
            self.get_logger().info("STM32 serial connected")
        except Exception as e:
            self.get_logger().error(f"Serial open error: {e}")
            raise SystemExit

        # ===== Timer =====
        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

        self.wheel_base = 0.42

    def loop(self):
        try:
            line = self.ser.readline().decode(errors="ignore").strip()
        except Exception:
            return

        if not line.startswith("vL"):
            return

        nums = re.findall(r"[-+]?\d*\.\d+|\d+", line)
        if len(nums) < 2:
            return

        vL = float(nums[0])
        vR = float(nums[1])

        vx = 0.5 * (vL + vR)
        wz = (vR - vL) / self.wheel_base

        now = self.get_clock().now().to_msg()

        # ===== Odometry (CHO EKF) =====
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # pose KHÔNG dùng
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0

        # velocity DÙNG
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz

        self.pub_odom.publish(odom)

        # ===== Twist (DEBUG) =====
        tw = Twist()
        tw.linear.x = vx
        tw.angular.z = wz
        self.pub_twist.publish(tw)

        self.get_logger().info(
            f"vx={vx:.3f} m/s  wz={wz:.3f} rad/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = STM32Receiver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
