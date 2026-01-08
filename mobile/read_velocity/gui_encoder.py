#!/usr/bin/env python3
import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def int32_from_u32(u: int) -> int:
    u &= 0xFFFFFFFF
    return u if u < 0x80000000 else u - 0x100000000

class STMBridge(Node):
    def __init__(self):
        super().__init__("stm_bridge")

        # ===== params =====
        self.declare_parameter("port", "/dev/ttyACM1")  # đúng cái đang đọc encoder
        self.declare_parameter("baud", 115200)

        self.declare_parameter("wheel_radius", 0.085)  # m
        self.declare_parameter("wheel_base", 0.35)    # m
        self.declare_parameter("gear_ratio", 1.0)
        self.declare_parameter("max_rpm", 300)

        self.declare_parameter("send_hz", 30.0)
        self.declare_parameter("timeout_stop_s", 0.5)

        self.declare_parameter("left_id", 0x01)
        self.declare_parameter("right_id", 0x02)
        self.declare_parameter("frame_id", "base_footprint")

        port = self.get_parameter("port").value
        baud = int(self.get_parameter("baud").value)
        self.R = float(self.get_parameter("wheel_radius").value)
        self.L = float(self.get_parameter("wheel_base").value)
        self.G = float(self.get_parameter("gear_ratio").value)
        self.max_rpm = int(self.get_parameter("max_rpm").value)
        self.send_period = 1.0 / float(self.get_parameter("send_hz").value)
        self.timeout_stop_s = float(self.get_parameter("timeout_stop_s").value)
        self.left_id = int(self.get_parameter("left_id").value)
        self.right_id = int(self.get_parameter("right_id").value)
        self.frame_id = self.get_parameter("frame_id").value

        # ===== serial =====
        self.ser = serial.Serial(port, baud, timeout=0.0, write_timeout=0.01)
        self.get_logger().info(f"Serial open: {port} @ {baud}")

        # ===== pubs =====
        self.pub_twist_cov = self.create_publisher(TwistWithCovarianceStamped, "/wheel_twist", 50)
        self.pub_twist = self.create_publisher(Twist, "/wheel_speed", 30)

        # ===== cmd_vel sub =====
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Twist, "/cmd_vel", self.cb_cmdvel, qos)

        self.target_rpm_l = 0
        self.target_rpm_r = 0
        self.last_cmd_time = 0.0
        self.last_sent = (None, None)

        # encoder state
        self.vL_mps = None
        self.vR_mps = None

        # timers
        self.create_timer(self.send_period, self.timer_send)
        self.create_timer(0.005, self.timer_read_serial)  # poll nhanh để không hụt line

    # ---- cmd_vel -> rpm ----
    def cb_cmdvel(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        v_l = v - (w * self.L * 0.5)
        v_r = v + (w * self.L * 0.5)

        rpm_l = (v_l / (2.0 * math.pi * self.R)) * 60.0
        rpm_r = (v_r / (2.0 * math.pi * self.R)) * 60.0
        rpm_l *= self.G
        rpm_r *= self.G

        self.target_rpm_l = int(clamp(round(rpm_l), -self.max_rpm, self.max_rpm))
        self.target_rpm_r = int(clamp(round(rpm_r), -self.max_rpm, self.max_rpm))
        self.last_cmd_time = self.get_clock().now().nanoseconds / 1e9

    def timer_send(self):
        now_s = self.get_clock().now().nanoseconds / 1e9
        if (now_s - self.last_cmd_time) > self.timeout_stop_s:
            rpm_l, rpm_r = 0, 0
        else:
            rpm_l, rpm_r = self.target_rpm_l, self.target_rpm_r

        # gửi khi thay đổi thôi (đỡ spam)
        if (rpm_l, rpm_r) == self.last_sent:
            return
        self.last_sent = (rpm_l, rpm_r)

        line = f"S,{rpm_l},{rpm_r}\n"
        try:
            self.ser.write(line.encode("ascii"))
        except Exception:
            pass

    # ---- read encoder from STM prints ----
    def timer_read_serial(self):
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

            # 0.001rpm -> m/s
            rpm = speed_001rpm / 1000.0
            omega = rpm * (2.0 * math.pi / 60.0)
            v_mps = omega * self.R

            if wheel_id == self.left_id:
                self.vL_mps = v_mps
            elif wheel_id == self.right_id:
                self.vR_mps = v_mps
            else:
                continue

            if self.vL_mps is None or self.vR_mps is None:
                continue

            vx = 0.5 * ((-self.vL_mps) + self.vR_mps)
            if abs(vx)<0.001:
                vx=0.0
            wz = (self.vR_mps + self.vL_mps) / self.L
            if not (math.isfinite(vx) and math.isfinite(wz)):
                continue

            stamp = self.get_clock().now().to_msg()

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

            dbg = Twist()
            dbg.linear.x = vx
            dbg.angular.z = wz
            self.pub_twist.publish(dbg)

    def parse_enc_line(self, line: str):
        # Expect: "ENC ID=01 RX 01 A4 70 76 00 d0 d1 d2 d3 cs"
        parts = line.split()
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

        try:
            rx_i = parts.index("RX")
        except ValueError:
            return None
        if len(parts) < rx_i + 1 + 10:
            return None

        try:
            frame = [int(x, 16) for x in parts[rx_i + 1: rx_i + 11]]
        except Exception:
            return None

        if frame[0] != wheel_id:
            return None
        if frame[1] != 0xA4:
            return None
        if frame[2] != 0x70 or frame[3] != 0x76:
            return None

        u = ((frame[5] << 24) | (frame[6] << 16) | (frame[7] << 8) | frame[8])
        return wheel_id, int32_from_u32(u)

def main():
    rclpy.init()
    node = STMBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
