#!/usr/bin/env python3
import time
import struct
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
import math

# ========= CONFIG =========
RXBUF = bytearray()
PORT   = "/dev/ch34x_can"
IDX_SPEED = (0x60F9, 0x19)
SER_BAUD = 2_000_000

NODE_L = 1
NODE_R = 2

SEND_HZ = 100

SCALE_DEC_PER_RPM = 167772.0 / 150.0 

wheel_radius = 0.085
wheel_base   = 0.35

left_sign  = -1.0
right_sign =  1.0

CMD_TIMEOUT = 0.5
MAX_RPM = 200.0

MISS_LIMIT = 50   # chống nhiễu (~0.5s)

# ========= CAN =========
def ws_build_std(can_id, data):
    t = 0xC0 | (len(data) & 0x0F)
    return bytes([0xAA, t, can_id & 0xFF, (can_id >> 8) & 0xFF]) + data + bytes([0x55])

def ws_write8(ser, can_id, data8):
    ser.write(ws_build_std(can_id, data8))

def ws_read_one(ser, timeout=0.01):
    t0 = time.time()
    while time.time() - t0 < timeout:
        chunk = ser.read(64)
        if chunk:
            RXBUF.extend(chunk)

        while RXBUF and RXBUF[0] != 0xAA:
            RXBUF.pop(0)

        if len(RXBUF) >= 4:
            t = RXBUF[1]
            dlc = t & 0x0F
            need = 1 + 1 + 2 + dlc + 1
            if len(RXBUF) >= need:
                pkt = bytes(RXBUF[:need])
                del RXBUF[:need]
                if pkt[-1] != 0x55:
                    continue
                can_id = pkt[2] | (pkt[3] << 8)
                data = pkt[4:4+dlc]
                return can_id, data
    return None

def sdo_send_read(ser, node, index, sub):
    req = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0,0,0,0])
    ws_write8(ser, 0x600 + node, req)

# ========= NEW: READ SDO =========
def sdo_read_u8(ser, node, index, sub):
    sdo_send_read(ser, node, index, sub)

    t_end = time.time() + 0.02
    while time.time() < t_end:
        got = ws_read_one(ser, timeout=0.001)
        if not got:
            continue

        cid, d = got
        if cid != 0x580 + node or len(d) != 8:
            continue

        if d[1] != (index & 0xFF) or d[2] != ((index >> 8) & 0xFF) or d[3] != sub:
            continue

        return d[4]

    return None

def sdo_read_u32(ser, node, index, sub):
    sdo_send_read(ser, node, index, sub)

    t_end = time.time() + 0.02
    while time.time() < t_end:
        got = ws_read_one(ser, timeout=0.001)
        if not got:
            continue

        cid, d = got
        if cid != 0x580 + node or len(d) != 8:
            continue

        if d[1] != (index & 0xFF) or d[2] != ((index >> 8) & 0xFF) or d[3] != sub:
            continue

        return struct.unpack("<I", d[4:8])[0]

    return None

def try_take_speed_reply(ser, node):
    expect_id = 0x580 + node
    idx_lo = IDX_SPEED[0] & 0xFF
    idx_hi = (IDX_SPEED[0] >> 8) & 0xFF
    sub    = IDX_SPEED[1]

    for _ in range(20):
        got = ws_read_one(ser, timeout=0.001)
        if not got:
            return None
        cid, d = got
        if cid != expect_id or len(d) != 8:
            continue
        if d[0] != 0x43:
            continue
        if d[1] != idx_lo or d[2] != idx_hi or d[3] != sub:
            continue

        raw = struct.unpack("<i", d[4:8])[0]
        return raw / 1000.0
    return None

def set_speed_dec_from_rpm(ser, node, rpm):
    dec = int(round(rpm * SCALE_DEC_PER_RPM))
    b = struct.pack("<i", dec)
    payload = bytes([0x23, 0xFF, 0x60, 0x00, b[0], b[1], b[2], b[3]])
    ws_write8(ser, 0x600 + node, payload)

def set_deceleration(ser, node, dec_value):
    # dec_value: giá trị DEC (ví dụ 150)
    b = struct.pack("<I", dec_value)  # U32 little-endian

    payload = bytes([
        0x23,                   # ghi 4 byte
        0x84, 0x60,             # index 0x6084
        0x00,                   # subindex
        b[0], b[1], b[2], b[3]
    ])

    ws_write8(ser, 0x600 + node, payload)

def sdo_write_u8(ser, node, index, sub, val):
    payload = bytes([0x2F, index & 0xFF, (index>>8)&0xFF, sub, val,0,0,0])
    ws_write8(ser, 0x600 + node, payload)

def sdo_write_u16(ser, node, index, sub, val):
    payload = bytes([0x2B, index & 0xFF, (index>>8)&0xFF, sub,
                     val & 0xFF, (val>>8)&0xFF, 0, 0])
    ws_write8(ser, 0x600 + node, payload)

def init_speed_mode(ser, node):
    sdo_write_u16(ser, node, 0x6040, 0x00, 0x0086)
    time.sleep(0.02)
    sdo_write_u8(ser, node, 0x6060, 0x00, 3)
    time.sleep(0.02)
    sdo_write_u16(ser, node, 0x6040, 0x00, 0x000F)
    time.sleep(0.02)

# ========= ROS =========
class CmdVelSub(Node):
    def __init__(self):
        super().__init__('can_cmdvel_sub')
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()

        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/wheel_twist', 50)
        self.frame_id = 'base_footprint'

    def cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last = time.time()

# ========= MAIN =========
def main():
    ser = serial.Serial(PORT, baudrate=SER_BAUD, timeout=0.001)
    time.sleep(0.5)

    init_speed_mode(ser, NODE_L)
    init_speed_mode(ser, NODE_R)

    # ===== SET DECELERATION =====
    set_deceleration(ser, NODE_L, 150)
    set_deceleration(ser, NODE_R, 150)

    time.sleep(0.05)

     # ===== READ DRIVER PARAM =====
    print("==== DRIVER PARAM CHECK ====")

    estop_L = sdo_read_u8(ser, NODE_L, 0x605A, 0x11)
    estop_R = sdo_read_u8(ser, NODE_R, 0x605A, 0x11)

    val_L = sdo_read_u32(ser, NODE_L, 0x605A, 0x01)
    val_R = sdo_read_u32(ser, NODE_R, 0x605A, 0x01)

    DEC_L = sdo_read_u32(ser, NODE_L, 0x6084, 0x00)
    DEC_R = sdo_read_u32(ser, NODE_R, 0x6084, 0x00)

    print(f"E-STOP L: {estop_L} | R: {estop_R}")
    print(f"0x605A01 L: {val_L} | R: {val_R}")
    print(f"0x608400 L: {DEC_L} | R: {DEC_R}")

    rclpy.init()
    cmd = CmdVelSub()

    dt = 1.0 / SEND_HZ
    next_t = time.monotonic()

    lastL = lastR = 0.0

    # ===== CAN state =====
    last_can_ok_time = time.time()
    can_connected = False
    miss_count = 0

    try:
        while True:
            rclpy.spin_once(cmd, timeout_sec=0.0)

            # ===== cmd timeout =====
            if time.time() - cmd.last > CMD_TIMEOUT:
                v = w = 0.0
            else:
                v = cmd.v
                w = cmd.w

            # ===== kinematics =====
            vL = v - w * wheel_base / 2.0
            vR = v + w * wheel_base / 2.0

            rpmL = (vL / (2 * math.pi * wheel_radius)) * 60.0 * left_sign
            rpmR = (vR / (2 * math.pi * wheel_radius)) * 60.0 * right_sign

            rpmL = max(-MAX_RPM, min(MAX_RPM, rpmL))
            rpmR = max(-MAX_RPM, min(MAX_RPM, rpmR))

            # ===== SEND =====
            if can_connected:
                set_speed_dec_from_rpm(ser, NODE_L, rpmL)
                set_speed_dec_from_rpm(ser, NODE_R, rpmR)
            else:
                set_speed_dec_from_rpm(ser, NODE_L, 0.0)
                set_speed_dec_from_rpm(ser, NODE_R, 0.0)

            # ===== READ =====
            sdo_send_read(ser, NODE_L, 0x60F9, 0x19)
            sdo_send_read(ser, NODE_R, 0x60F9, 0x19)

            t_end = time.time() + 0.004
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

            # ===== CAN CHECK =====
            if gotL and gotR:
                miss_count = 0
                last_can_ok_time = time.time()

                if not can_connected:
                    print("✅ CAN connect OK")
                    can_connected = True
            else:
                miss_count += 1

                if (time.time() - last_can_ok_time > 5.0) and can_connected:
                    print("🚨 LOST CAN >5s - STOP motor")
                    can_connected = False

                    set_speed_dec_from_rpm(ser, NODE_L, 0.0)
                    set_speed_dec_from_rpm(ser, NODE_R, 0.0)

            # ===== DEBUG =====
            if abs(lastL) < 0.03: lastL = 0
            if abs(lastR) < 0.03: lastR = 0
            print(f"L={lastL:.2f} rpm | R={lastR:.2f} rpm")

            # ===== ODOM =====
            omegaL = (lastL * 2 * math.pi) / 60.0
            omegaR = (lastR * 2 * math.pi) / 60.0

            vL_m = omegaL * wheel_radius
            vR_m = omegaR * wheel_radius

            vx = 0.5 * (-vL_m + vR_m)
            wz = (vR_m + vL_m) / wheel_base

            msg = TwistWithCovarianceStamped()
            msg.header.stamp = cmd.get_clock().now().to_msg()
            msg.header.frame_id = cmd.frame_id
            msg.twist.twist.linear.x = vx
            msg.twist.twist.angular.z = wz

            cov = [0.0]*36
            cov[0] = 0.02
            cov[35] = 0.05
            msg.twist.covariance = cov

            cmd.pub.publish(msg)

            next_t += dt
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

    except KeyboardInterrupt:
        print("Stopping...")
        set_speed_dec_from_rpm(ser, NODE_L, 0)
        set_speed_dec_from_rpm(ser, NODE_R, 0)
        ser.close()

if __name__ == "__main__":
    main()