#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu

import math


class FakeEkfInput(Node):
    def __init__(self):
        super().__init__("fake_ekf_input")

        # ===== Publishers =====
        self.pub_twist = self.create_publisher(
            TwistWithCovarianceStamped,
            "/wheel_twist",
            10
        )

        self.pub_imu = self.create_publisher(
            Imu,
            "/imu/data",
            10
        )

        # ===== Timer =====
        self.timer = self.create_timer(0.02, self.loop)  # 50 Hz
        self.start_time = self.get_clock().now()

        self.get_logger().info("Fake EKF input node started")

    def loop(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        # =========================
        # Fake ENCODER (vx + wz)
        # =========================
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = now.to_msg()
        twist.header.frame_id = "base_footprint"

        twist.twist.twist.linear.x = 0.2    # m/s
        twist.twist.twist.angular.z = 0.1   # rad/s

        SMALL = 0.01
        BIG = 1e3

        twist.twist.covariance = [
            SMALL, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   BIG,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   BIG,   0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   BIG,   0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   BIG,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   SMALL
        ]

        self.pub_twist.publish(twist)

        # =========================
        # Fake IMU (yaw)
        # =========================
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "base_footprint"

        yaw = 0.1 * t  # quay đều

        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = math.sin(yaw / 2.0)
        imu.orientation.w = math.cos(yaw / 2.0)

        imu.orientation_covariance = [
            0.01,0.0,0.0,
            0.0,0.01,0.0,
            0.0,0.0,0.01
        ]

        # Không dùng gyro / accel
        imu.angular_velocity_covariance[0] = -1
        imu.linear_acceleration_covariance[0] = -1

        self.pub_imu.publish(imu)


def main():
    rclpy.init()
    node = FakeEkfInput()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()