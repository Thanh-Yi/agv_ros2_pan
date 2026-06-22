#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import math

class AdaptiveAccel(Node):
    def __init__(self):
        super().__init__('adaptive_accel')

        # subscribe vận tốc
        self.sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # client để set param cho velocity_smoother
        self.cli = self.create_client(SetParameters, '/velocity_smoother/set_parameters')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /velocity_smoother/set_parameters service...')

        self.current_mode = None

        self.get_logger().info("Adaptive Accel Node Started")

    def set_param(self, name, values):
        req = SetParameters.Request()

        param = Parameter()
        param.name = name

        val = ParameterValue()
        val.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        val.double_array_value = values

        param.value = val
        req.parameters.append(param)

        self.cli.call_async(req)

    def odom_callback(self, msg):
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        v = math.sqrt(vx**2 + vy**2)

        # ===== hysteresis chống rung =====
        if self.current_mode == "LOW_SPEED":
            if v > 0.5:
                mode = "HIGH_SPEED"
            else:
                mode = "LOW_SPEED"
        elif self.current_mode == "HIGH_SPEED":
            if v < 0.4:
                mode = "LOW_SPEED"
            else:
                mode = "HIGH_SPEED"
        else:
            mode = "LOW_SPEED"

        # ===== set thông số =====
        if mode == "LOW_SPEED":
            accel = [0.1, 0.0, 0.5]
            decel = [-0.1, 0.0, -0.5]
        else:
            accel = [0.1, 0.0, 0.5]
            decel = [-0.05, 0.0, -0.5]

        # chỉ update khi đổi mode
        if mode != self.current_mode:
            self.current_mode = mode

            self.get_logger().info(f"Switch to {mode}, v={v:.2f}")

            # set cả accel và decel
            self.set_param('max_accel', accel)
            self.set_param('max_decel', decel)


def main():
    rclpy.init()
    node = AdaptiveAccel()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()