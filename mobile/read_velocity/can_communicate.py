#!/usr/bin/env python3
import time
import struct
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped 
from geometry_msgs.msg import TwistWithCovarianceStamped
import math

# ========= CONFIG =========
RXBUF = bytearray()
INBOX = {}   # key: can_id, value: list of data(bytes)
PORT   = "/dev/ch34x_can"     # đổi nếu USB-CAN-A là ttyUSB1/2...
IDX_SPEED = (0x60F9, 0x19)
SER_BAUD = 2_000_000        # Waveshare USB-CAN-A serial baud thường 2M

NODE_L = 1                  # bánh trái (CANopen node id)
NODE_R = 2                  # bánh phải (node id = motor1 + 1)

RPM_L  = -25.0               # <-- bạn tự set
RPM_R  = 25.0               # <-- bạn tự set
SEND_HZ = 100                # gửi 20Hz cho chắc

# 150 rpm ↔ 167772 DEC (từ ví dụ bạn đã chạy trên app)
SCALE_DEC_PER_RPM = 167772.0 / 150.0 

wheel_radius = 0.085   # m
wheel_base   = 0.35    # m

left_sign  = -1.0      # giống bạn đang phải đảo trái
right_sign =  1.0

CMD_TIMEOUT = 0.5      # nếu quá 0.3s ko có cmd_vel thì stop
MAX_RPM = 200.0        # giới hạn an toàn

def sdo_send_read(ser, node, index, sub):
    req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0,0,0,0])
    ws_write8(ser, 0x600 + node, req)

def try_take_speed_reply(ser, node):
    """Non-blocking-ish: scan a bit for the specific 0x43 index/sub reply, return rpm or None."""
    expect_id = 0x580 + node
    idx_lo = IDX_SPEED[0] & 0xFF
    idx_hi = (IDX_SPEED[0] >> 8) & 0xFF
    sub    = IDX_SPEED[1]

    # poll a few packets quickly
    for _ in range(20):
        got = ws_read_one(ser, timeout=0.001)  # timeout=0 -> không chờ
        if not got:
            return None
        cid, d = got
        if cid != expect_id or len(d) != 8:
            continue
        # chỉ nhận đúng READ response của speed
        if d[0] != 0x43: 
            continue
        if d[1] != idx_lo or d[2] != idx_hi or d[3] != sub:
            continue

        raw = struct.unpack("<i", d[4:8])[0]
        return raw / 1000.0

    return None

def rpm_to_dec(rpm: float) -> int:
    return int(round(rpm * SCALE_DEC_PER_RPM))

# ========= Waveshare USB-CAN-A variable-length packet (standard frame) =========
def ws_build_std(can_id: int, data: bytes) -> bytes:
    assert 0 <= can_id <= 0x7FF
    assert 0 <= len(data) <= 8
    t = 0xC0 | (len(data) & 0x0F)   # standard data frame + dlc
    return bytes([0xAA, t, can_id & 0xFF, (can_id >> 8) & 0xFF]) + data + bytes([0x55])

def ws_write8(ser: serial.Serial, can_id: int, data8: bytes):
    assert len(data8) == 8
    ser.write(ws_build_std(can_id, data8))

def ws_read_one(ser: serial.Serial, timeout=0.01):
    """Return (can_id:int, data:bytes) or None. Uses global RXBUF."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        chunk = ser.read(64)
        if chunk:
            RXBUF.extend(chunk)

        # resync to 0xAA
        while RXBUF and RXBUF[0] != 0xAA:
            RXBUF.pop(0)

        if len(RXBUF) >= 4:
            t = RXBUF[1]
            dlc = t & 0x0F
            need = 1 + 1 + 2 + dlc + 1  # AA TYPE ID2 DATA DLC 55
            if len(RXBUF) >= need:
                pkt = bytes(RXBUF[:need])
                del RXBUF[:need]
                if pkt[-1] != 0x55:
                    continue
                can_id = pkt[2] | (pkt[3] << 8)
                data = pkt[4:4+dlc]
                return can_id, data

    return None

def inbox_put(can_id, data):
    INBOX.setdefault(can_id, []).append(data)

def inbox_get(can_id):
    lst = INBOX.get(can_id)
    if lst:
        return lst.pop(0)
    return None

def sdo_read_i32(ser, node, index, sub, timeout=0.01):
    expect_id = 0x580 + node
    idx_lo = index & 0xFF
    idx_hi = (index >> 8) & 0xFF

    # gửi request
    req = bytes([0x40, idx_lo, idx_hi, sub, 0,0,0,0])
    ws_write8(ser, 0x600 + node, req)

    deadline = time.time() + timeout
    while time.time() < deadline:
        got = ws_read_one(ser, timeout=0.01)
        if not got:
            continue

        cid, d = got

        # không đúng node thì bỏ qua (hoặc inbox nếu bạn muốn)
        if cid != expect_id or len(d) != 8:
            continue

        # chỉ nhận đúng expedited 4-byte upload response
        # và đúng index/sub (để khỏi dính ACK 0x60)
        if d[0] != 0x43:
            continue
        if d[1] != idx_lo or d[2] != idx_hi or d[3] != sub:
            continue

        return struct.unpack("<i", d[4:8])[0]

    return None

def read_pos_raw(ser, node: int):
    # Actual position: 0x6063:00
    return sdo_read_i32(ser, node, 0x6063, 0x00)


def read_speed_rpm(ser, node: int):
    # Actual speed_0.001rpm: 0x60F9:19
    raw = sdo_read_i32(ser, node, 0x60F9, 0x19)
    if raw is None:
        return None
    return raw / 1000.0

# ========= CANopen SDO writes =========
def sdo_write_u8(ser, node, index, sub, val_u8):
    payload = bytes([0x2F, index & 0xFF, (index>>8) & 0xFF, sub, val_u8 & 0xFF, 0, 0, 0])
    ws_write8(ser, 0x600 + node, payload)

def sdo_write_u16(ser, node, index, sub, val_u16):
    lo = val_u16 & 0xFF
    hi = (val_u16 >> 8) & 0xFF
    payload = bytes([0x2B, index & 0xFF, (index>>8) & 0xFF, sub, lo, hi, 0, 0])
    ws_write8(ser, 0x600 + node, payload)

def sdo_write_i32(ser, node, index, sub, val_i32):
    b = struct.pack("<i", int(val_i32))
    payload = bytes([0x23, index & 0xFF, (index>>8) & 0xFF, sub, b[0], b[1], b[2], b[3]])
    ws_write8(ser, 0x600 + node, payload)

# ========= Driver init for speed mode =========
def init_speed_mode(ser, node):
    # Clear error: 0x6040 = 0x86
    sdo_write_u16(ser, node, 0x6040, 0x00, 0x0086)
    time.sleep(0.02)

    # Operation mode: 0x6060 = 3 (speed mode)
    sdo_write_u8(ser, node, 0x6060, 0x00, 3)
    time.sleep(0.02)

    # Enable: 0x6040 = 0x0F
    sdo_write_u16(ser, node, 0x6040, 0x00, 0x000F)
    time.sleep(0.02)

# def set_rpm(ser, node, rpm):
#     dec = rpm_to_dec(rpm)
#     # Target velocity DEC: 0x60FF:00
#     sdo_write_i32(ser, node, 0x60FF, 0x00, dec)

# def set_rpm(ser, node, rpm: int):
#     # Target Velocity_rpm: Index 0x2FF0, Sub 0x09 (16S, rpm)
#     # SDO write 16-bit: 0x2B
#     val = int(rpm)
#     if val < -32768: val = -32768
#     if val >  32767: val =  32767
#     u16 = val & 0xFFFF
#     lo = u16 & 0xFF
#     hi = (u16 >> 8) & 0xFF
#     payload = bytes([0x2B, 0xF0, 0x2F, 0x09, lo, hi, 0, 0])
#     ws_write8(ser, 0x600 + node, payload)

def set_speed_dec_from_rpm(ser, node, rpm_float: float):
    # giữ rpm dạng float tới phút cuối để mượt tốc thấp
    dec = int(round(rpm_float * SCALE_DEC_PER_RPM))
    b = struct.pack("<i", dec)
    payload = bytes([0x23, 0xFF, 0x60, 0x00, b[0], b[1], b[2], b[3]])  # 0x60FF:00
    ws_write8(ser, 0x600 + node, payload)

class CmdVelSub(Node):
    def __init__(self):
        super().__init__('can_cmdvel_sub')
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        self.last = time.time()

class CmdVelSub(Node):
    def __init__(self):
        super().__init__('can_cmdvel_sub')
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

        # publisher cho EKF
        self.pub_twist_cov = self.create_publisher(
            TwistWithCovarianceStamped, '/wheel_twist', 50
        )
        self.frame_id = 'base_footprint'  # nếu muốn param hóa thì làm sau

    def cb(self, msg: Twist):
        self.v = float(msg.linear.x)
        self.w = float(msg.angular.z)
        self.last = time.time()

def main():
    ser = serial.Serial(PORT, baudrate=SER_BAUD, timeout=0.001)
    time.sleep(0.5)

    print("Init nodes...")
    init_speed_mode(ser, NODE_L)
    init_speed_mode(ser, NODE_R)
    rclpy.init()
    cmd = CmdVelSub()

    print(f"Run fixed RPM: L={RPM_L} rpm, R={RPM_R} rpm  (Ctrl+C to stop)")
    dt = 1.0 / SEND_HZ 
    next_t = time.monotonic()
    last_t = None
    ok_count = 0
    t0 = time.time()
    lastL = 0.0
    lastR = 0.0

    try:
        #cnt = 0
        while True:
            rclpy.spin_once(cmd,timeout_sec=0.0)
            # nếu mất cmd_vel quá lâu thì stop
            if time.time() - cmd.last > CMD_TIMEOUT:
                v = 0.0
                w = 0.0
            else:
                v = cmd.v
                w = cmd.w

            # diff-drive kinematics
            vL = v - w * wheel_base / 2.0
            vR = v + w * wheel_base / 2.0

            # m/s -> rpm
            rpmL = (vL / (2.0 * 3.1415926535 * wheel_radius)) * 60.0
            rpmR = (vR / (2.0 * 3.1415926535 * wheel_radius)) * 60.0

            rpmL *= left_sign
            rpmR *= right_sign

            # clamp
            rpmL = max(-MAX_RPM, min(MAX_RPM, rpmL))
            rpmR = max(-MAX_RPM, min(MAX_RPM, rpmR))
            # set_rpm(ser, NODE_L, int(round(rpmL)))
            # set_rpm(ser, NODE_R, int(round(rpmR)))
            set_speed_dec_from_rpm(ser, NODE_L,rpmL)
            time.sleep(0.001)
            set_speed_dec_from_rpm(ser, NODE_R,rpmR)
            time.sleep(0.001)

            # READ_EVERY = 1 (mỗi vòng đọc)
            sdo_send_read(ser, NODE_L, 0x60F9, 0x19)
            sdo_send_read(ser, NODE_R, 0x60F9, 0x19)

            t_end = time.time() + 0.004  # 8ms cửa sổ hứng reply
            gotL = gotR = False
            while time.time() < t_end and (not gotL or not gotR):
                spL = try_take_speed_reply(ser, NODE_L)
                if spL is not None:
                    lastL = spL
                    gotL = True

                spR = try_take_speed_reply(ser, NODE_R)
                if spR is not None:
                    lastR = spR
                    gotR = True

            if abs(lastL) <= 0.03 : lastL = 0
            if abs(lastR) <= 0.03 : lastR = 0
            print(f"L_sp={lastL:.3f} rpm | R_sp={lastR:.3f} rpm")
            # ===== wheel rpm -> robot twist (m/s, rad/s) =====
            # lastL,lastR đang là rpm (wheel rpm). đổi ra m/s của từng bánh:
            omegaL = (lastL * 2.0 * math.pi) / 60.0
            omegaR = (lastR * 2.0 * math.pi) / 60.0
            vL_mps = omegaL * wheel_radius
            vR_mps = omegaR * wheel_radius

            # diff-drive: vx, wz
            vx = 0.5 * (-vL_mps + vR_mps)
            wz = (vR_mps + vL_mps) / wheel_base

            # optional: deadband nhỏ cho sạch
            if abs(vx) < 0.001: vx = 0.0
            if abs(wz) < 0.001: wz = 0.0

            # ===== đóng gói TwistWithCovarianceStamped cho EKF =====
            msg = TwistWithCovarianceStamped()
            msg.header.stamp = cmd.get_clock().now().to_msg()
            msg.header.frame_id = cmd.frame_id

            msg.twist.twist.linear.x = float(vx)
            msg.twist.twist.linear.y = 0.0
            msg.twist.twist.linear.z = 0.0
            msg.twist.twist.angular.x = 0.0
            msg.twist.twist.angular.y = 0.0
            msg.twist.twist.angular.z = float(wz)

            # covariance order: [vx,vy,vz, wx,wy,wz]
            VX_VAR = 0.02
            WZ_VAR = 0.05
            BIG = 1e3
            cov = [0.0] * 36
            cov[0]  = VX_VAR
            cov[7]  = BIG
            cov[14] = BIG
            cov[21] = BIG
            cov[28] = BIG
            cov[35] = WZ_VAR
            msg.twist.covariance = cov

            cmd.pub_twist_cov.publish(msg)

            next_t += dt
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(0.0001)
            else:
                # nếu bị trễ thì reset để khỏi trượt pha mãi
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("Stopping...")
        set_speed_dec_from_rpm(ser, NODE_L, 0.0)
        set_speed_dec_from_rpm(ser, NODE_R, 0.0)
        time.sleep(0.01)
        ser.close()
        print("Stopped.")

if __name__ == "__main__":
    main()
