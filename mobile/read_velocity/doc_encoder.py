#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped, Twist


def int32_from_u32(u: int) -> int:
    """convert unsigned 32-bit to signed int32"""
    u &= 0xFFFFFFFF
    return u if u < 0x80000000 else u - 0x100000000


class STM32ReceiverENC(Node):
    def __init__(self):
        super().__init__("stm32_receiver_enc7076")

        # ===== ROS params =====
        self.declare_parameter("port", "/dev/cp210x_stm")  # hoặc /dev/ttyTHS1, /dev/ttyUSB0
        self.declare_parameter("baud", 115200)
        self.declare_parameter("wheel_radius", 0.05)       # m
        self.declare_parameter("wheel_base", 0.42)         # m
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("left_id", 0x01)
        self.declare_parameter("right_id", 0x02)
        self.declare_parameter("startup_ignore_frames", 6) # bỏ vài frame đầu
        self.declare_parameter("serial_poll_hz", 200.0)    # đọc serial nhanh hơn tốc độ STM in

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.R = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_base").value)
        self.frame_id = self.get_parameter("frame_id").value
        self.left_id = int(self.get_parameter("left_id").value)
        self.right_id = int(self.get_parameter("right_id").value)
        self.ignore_n = int(self.get_parameter("startup_ignore_frames").value)
        poll_hz = float(self.get_parameter("serial_poll_hz").value)

        # ===== Publishers =====
        self.pub_twist_cov = self.create_publisher(TwistWithCovarianceStamped, "/wheel_twist", 50)
        self.pub_twist = self.create_publisher(Twist, "/wheel_speed", 30)

        # ===== Serial open =====
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.0)
            self.get_logger().info(f"Serial connected: {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().fatal(f"Serial open error: {e}")
            raise SystemExit

        # ===== State =====
        self.stable_count = 0
        self.vL_mps = None
        self.vR_mps = None
        self.last_update_ns_L = 0
        self.last_update_ns_R = 0

        self.timer = self.create_timer(1.0 / poll_hz, self.loop)

    def destroy_node(self):
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

    def loop(self):
        # đọc hết các line đang có trong buffer (non-blocking)
        while True:
            try:
                raw = self.ser.readline()
            except Exception:
                return

            if not raw:
                return

            line = raw.decode(errors="ignore").strip()
            if not line.startswith("ENC"):
                continue

            parsed = self.parse_enc_line(line)
            if parsed is None:
                continue

            wheel_id, speed_001rpm = parsed

            # startup filter
            self.stable_count += 1
            if self.stable_count < self.ignore_n:
                continue

            # đổi 0.001rpm -> m/s
            # rpm = speed_001rpm / 1000
            rpm = speed_001rpm / 1000.0
            omega = rpm * (2.0 * math.pi / 60.0)   # rad/s
            v_mps = omega * self.R                 # m/s

            now = self.get_clock().now()
            now_ns = now.nanoseconds

            if wheel_id == self.left_id:
                self.vL_mps = v_mps
                self.last_update_ns_L = now_ns
            elif wheel_id == self.right_id:
                self.vR_mps = v_mps
                self.last_update_ns_R = now_ns
            else:
                continue

            # publish khi đã có đủ cả 2 bánh (dùng latest)
            if self.vL_mps is None or self.vR_mps is None:
                continue

            # optional: nếu 1 bánh quá lâu không update thì bỏ
            # (ví dụ > 0.5s)
            if (now_ns - self.last_update_ns_L) > 500_000_000:
                continue
            if (now_ns - self.last_update_ns_R) > 500_000_000:
                continue

            vx = 0.5 * (self.vL_mps + self.vR_mps)
            wz = (self.vR_mps - self.vL_mps) / self.L

            if not (math.isfinite(vx) and math.isfinite(wz)):
                continue

            stamp = now.to_msg()

            # ===== TwistWithCovarianceStamped =====
            msg = TwistWithCovarianceStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.twist.twist.linear.x = vx
            msg.twist.twist.angular.z = wz

            SMALL = 0.02
            BIG = 1e3
            msg.twist.covariance = [
                SMALL, 0.0,   0.0,   0.0,   0.0,   0.0,
                0.0,   BIG,   0.0,   0.0,   0.0,   0.0,
                0.0,   0.0,   BIG,   0.0,   0.0,   0.0,
                0.0,   0.0,   0.0,   BIG,   0.0,   0.0,
                0.0,   0.0,   0.0,   0.0,   BIG,   0.0,
                0.0,   0.0,   0.0,   0.0,   0.0,   SMALL
            ]
            self.pub_twist_cov.publish(msg)

            # ===== DEBUG Twist =====
            dbg = Twist()
            dbg.linear.x = vx
            dbg.angular.z = wz
            self.get_logger().info(
            f"vx={vx:.3f} m/s  wz={wz:.3f} rad/s"
            )
            self.pub_twist.publish(dbg)

    def parse_enc_line(self, line: str):
        """
        Expect format from STM:
        'ENC ID=01 RX 01 A4 70 76 00 d0 d1 d2 d3 cs'
        Return (wheel_id_int, speed_001rpm_int32) or None
        """
        parts = line.split()
        # minimal sanity
        if len(parts) < 4:
            return None

        # find ID=xx
        wheel_id = None
        for p in parts:
            if p.startswith("ID="):
                try:
                    wheel_id = int(p[3:], 16)
                except Exception:
                    return None
                break
        if wheel_id is None:
            return None

        # find "RX" index and read next 10 bytes
        try:
            rx_i = parts.index("RX")
        except ValueError:
            return None

        if len(parts) < rx_i + 1 + 10:
            return None

        hexbytes = parts[rx_i + 1: rx_i + 1 + 10]
        try:
            frame = [int(x, 16) for x in hexbytes]
        except Exception:
            return None

        # Filter đúng reply encoder 0x7076: ID, A4, 70, 76
        if frame[0] != wheel_id:
            return None
        if frame[1] != 0xA4:
            return None
        if frame[2] != 0x70 or frame[3] != 0x76:
            return None

        # data bytes: frame[5..8]
        u = ((frame[5] << 24) |
             (frame[6] << 16) |
             (frame[7] << 8) |
             (frame[8] << 0))
        speed_001rpm = int32_from_u32(u)
        return wheel_id, speed_001rpm


def main(args=None):
    rclpy.init(args=args)
    node = STM32ReceiverENC()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
