#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped, Twist

import serial
import re
import math


class STM32Receiver(Node):
    def __init__(self):
        super().__init__("stm32_receiver")

        # ===== Publishers =====
        self.pub_twist_cov = self.create_publisher(
            TwistWithCovarianceStamped,
            "/wheel_twist",
            50
        )

        # (optional) debug
        self.pub_twist = self.create_publisher(
            Twist,
            "/wheel_speed",
            30
        )

        # ===== Serial =====
        try:
            self.ser = serial.Serial(
                "/dev/cp210x_stm", 115200, timeout=0
            )
            self.get_logger().info("STM32 serial connected")
        except Exception as e:
            self.get_logger().fatal(f"Serial open error: {e}")
            raise SystemExit

        # ===== Timer =====
        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz

        self.wheel_base = 0.42
        self.stable_count = 0

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

        try:
            vL = float(nums[0])
            vR = float(nums[1])
        except ValueError:
            return

        if not (math.isfinite(vL) and math.isfinite(vR)):
            return

        vx = 0.5 * (vL + vR)
        wz = (vR - vL) / self.wheel_base

        if not (math.isfinite(vx) and math.isfinite(wz)):
            return

        # startup filter (tránh frame rác lúc đầu)
        self.stable_count += 1
        if self.stable_count < 10:
            return

        now = self.get_clock().now().to_msg()
        if now.sec == 0 and now.nanosec == 0:
            return

        # ===== TwistWithCovarianceStamped =====
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = now
        msg.header.frame_id = "base_footprint"

        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = wz

        SMALL = 0.02
        BIG   = 1e3

        msg.twist.covariance = [
            SMALL, 0.0,   0.0,   0.0,   0.0,   0.0,   # vx
            0.0,   BIG,   0.0,   0.0,   0.0,   0.0,   # vy
            0.0,   0.0,   BIG,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   BIG,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   BIG,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   SMALL # wz
        ]

        self.pub_twist_cov.publish(msg)

        # ===== DEBUG Twist =====
        dbg = Twist()
        dbg.linear.x = vx
        dbg.angular.z = wz
        self.pub_twist.publish(dbg)

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

