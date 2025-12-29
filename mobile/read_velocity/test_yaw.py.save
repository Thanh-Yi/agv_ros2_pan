#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RotateForTime(Node):
    def __init__(self):
        super().__init__('rotate_for_time')

        # --- Config ---
        self.topic = '/cmd_vel'
        self.angular_z = math.pi / 5.0   # rad/s
        self.duration_s = 10.0           # seconds (exact by monotonic clock)
        self.publish_hz = 30.0           # publish rate

        self.pub = self.create_publisher(Twist, self.topic, 10)

        self.start_t = time.monotonic()
        self.stopped = False

        period = 1.0 / self.publish_hz
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f'Rotating: angular_z={self.angular_z:.6f} rad/s for {self.duration_s:.3f}s on {self.topic}'
        )

    def on_timer(self):
        elapsed = time.monotonic() - self.start_t

        msg = Twist()

        if elapsed < self.duration_s:
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(self.angular_z)
            self.pub.publish(msg)
            return

        # Past duration -> stop exactly once, then shutdown
        if not self.stopped:
            msg.angular.z = 0.0
            self.pub.publish(msg)
            self.get_logger().info('Done. Stopped cmd_vel (published 0) and exiting.')
            self.stopped = True
            # give a tiny moment for message to go out
            time.sleep(0.05)
            rclpy.shutdown()


def main():
    rclpy.init()
    node = RotateForTime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # If Ctrl+C: also stop the robot
        msg = Twist()
        node.pub.publish(msg)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

