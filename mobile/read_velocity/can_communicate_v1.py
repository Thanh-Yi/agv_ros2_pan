#!/usr/bin/env python3
import time
import struct
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped
import math

# ========= CONFIG =========
PORT = "/dev/ch34x_can"
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

# ===== ramp =====
MAX_ACCEL = 0.5 #0.7

# ===== balance =====
KP_BALANCE = 0.1

# ===== SAFETY =====
ERROR_TIMEOUT = 0.5
RPM_DIFF_ERR  = 30.0

last_ok_time = time.time()
error_flag = False

# ===== ERROR MAP =====
ERROR_BITS = {
    0:  "Internal error",
    1:  "Encoder ABZ signal error",
    2:  "Encoder UVW signal error",
    3:  "Encoder counting error",
    4:  "Driver temperature too high",
    5:  "Driver bus voltage too high",
    6:  "Driver bus voltage too low",
    7:  "Driver output short-circuit",
    8:  "Braking resistor temperature too high",
    9:  "Following error over-range",
    10: "Reserved",
    11: "I2T error (Overload)",
    12: "Speed following error over-range",
    13: "Motor temperature too high",
    14: "Searching motor failed (encoder comm)",
    15: "Communication failed"
}

# ========= CAN =========
def ws_build_std(can_id: int, data: bytes) -> bytes:
    t = 0xC0 | (len(data) & 0x0F)
    return bytes([0xAA, t, can_id & 0xFF, (can_id >> 8) & 0xFF]) + data + bytes([0x55])

def ws_write8(ser, can_id, data8):
    global error_flag, last_ok_time
    try:
        ser.write(ws_build_std(can_id, data8))
        last_ok_time = time.time()
    except serial.SerialException:
        print("❌ CAN/Serial ERROR!")
        error_flag = True

# ========= SDO WRITE =========
def sdo_write_i32(ser, node, index, sub, val):
    b = struct.pack("<i", int(val))
    payload = bytes([0x23, index & 0xFF, (index>>8)&0xFF, sub, b[0], b[1], b[2], b[3]])
    ws_write8(ser, 0x600 + node, payload)

def sdo_write_u16(ser, node, index, sub, val):
    payload = bytes([0x2B, index & 0xFF, (index>>8)&0xFF, sub, val&0xFF, (val>>8)&0xFF, 0,0])
    ws_write8(ser, 0x600 + node, payload)

def sdo_write_u8(ser, node, index, sub, val):
    payload = bytes([0x2F, index & 0xFF, (index>>8)&0xFF, sub, val,0,0,0])
    ws_write8(ser, 0x600 + node, payload)

# ========= SET ACC/DEC =========
def set_acc_dec(ser, node):
    # Acceleration
    sdo_write_i32(ser, node, 0x6083, 0x00, 160)

    # Deceleration
    sdo_write_i32(ser, node, 0x6084, 0x00, 160)

    # Emergency deceleration (sub-index 1)
    sdo_write_i32(ser, node, 0x605A, 0x01, 350)
# ========= SDO READ =========
def sdo_read_i32(ser, node, index, sub):
    global error_flag

    payload = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0])
    ws_write8(ser, 0x600 + node, payload)

    time.sleep(0.01)

    try:
        data = ser.read(32)
        if len(data) < 8:
            return None

        for i in range(len(data) - 11):
            if data[i] == 0xAA:
                can_id = data[i+2] | (data[i+3] << 8)
                if can_id == (0x580 + node):
                    cmd = data[i+4]
                    if cmd in [0x43, 0x4B, 0x4F]:
                        b0 = data[i+8]
                        b1 = data[i+9]
                        b2 = data[i+10]
                        b3 = data[i+11]
                        val = struct.unpack("<I", bytes([b0, b1, b2, b3]))[0]
                        return val
        return None

    except:
        error_flag = True
        return None

# ========= ERROR DECODE =========
def decode_error_bits(err_code):
    if err_code == 0:
        return ["OK"]

    errors = []
    for bit, desc in ERROR_BITS.items():
        if err_code & (1 << bit):
            errors.append(f"BIT{bit}: {desc}")
    return errors

def read_driver_error(ser, node):
    err = sdo_read_i32(ser, node, 0x2601, 0x00)
    if err is None:
        return None

    if err != 0:
        print(f"\n❌ Driver {node} ERROR CODE: {hex(err)}")
        for e in decode_error_bits(err):
            print("   ->", e)

    return err

# ========= INIT =========
def init_speed_mode(ser, node):
    sdo_write_u16(ser, node, 0x6040, 0x00, 0x0086)
    time.sleep(0.02)
    sdo_write_u8(ser, node, 0x6060, 0x00, 3)
    time.sleep(0.02)
    sdo_write_u16(ser, node, 0x6040, 0x00, 0x000F)
    time.sleep(0.02)

def set_speed_dec_from_rpm(ser, node, rpm):
    dec = int(round(rpm * SCALE_DEC_PER_RPM))
    b = struct.pack("<i", dec)
    payload = bytes([0x23, 0xFF, 0x60, 0x00, b[0], b[1], b[2], b[3]])
    ws_write8(ser, 0x600 + node, payload)

# ========= EMERGENCY =========
def emergency_stop(ser):
    print("🛑 EMERGENCY STOP")
    try:
        set_speed_dec_from_rpm(ser, NODE_L, 0)
        set_speed_dec_from_rpm(ser, NODE_R, 0)
    except:
        pass

# ========= ROS =========
class CmdVelSub(Node):
    def __init__(self):
        super().__init__('cmd_node')
        self.v = 0.0
        self.w = 0.0
        self.last = time.time()
        self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub_twist_cov = self.create_publisher(
            TwistWithCovarianceStamped, '/wheel_twist', 50
        )
        self.frame_id = 'base_footprint'

    def cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z
        self.last = time.time()

# ========= MAIN =========
def main():
    global error_flag

    ser = serial.Serial(PORT, baudrate=SER_BAUD, timeout=0.001)
    time.sleep(0.5)

    print("Init motor...")
    init_speed_mode(ser, NODE_L)
    init_speed_mode(ser, NODE_R)

    # 🔥 THÊM ĐOẠN NÀY
    set_acc_dec(ser, NODE_L)
    set_acc_dec(ser, NODE_R)

    rclpy.init()
    node = CmdVelSub()

    dt = 1.0 / SEND_HZ

    rpmL_cur = 0.0
    rpmR_cur = 0.0

    next_t = time.monotonic()

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.0)

            # ===== target =====
            if time.time() - node.last > CMD_TIMEOUT:
                target_v = 0.0
                target_w = 0.0
            else:
                target_v = node.v
                target_w = node.w

            # ===== kinematics =====
            vL_t = target_v - target_w * wheel_base / 2
            vR_t = target_v + target_w * wheel_base / 2

            rpmL_t = (vL_t / (2*math.pi*wheel_radius)) * 60 * left_sign
            rpmR_t = (vR_t / (2*math.pi*wheel_radius)) * 60 * right_sign

            rpmL_t = max(-MAX_RPM, min(MAX_RPM, rpmL_t))
            rpmR_t = max(-MAX_RPM, min(MAX_RPM, rpmR_t))

            # ===== ramp =====
            max_step = MAX_ACCEL * dt * 60

            dL = rpmL_t - rpmL_cur
            dR = rpmR_t - rpmR_cur

            dL = max(-max_step, min(max_step, dL))
            dR = max(-max_step, min(max_step, dR))

            rpmL_cur += dL
            rpmR_cur += dR

            # ===== balance =====
            error = rpmL_cur - rpmR_cur
            correction = KP_BALANCE * error

            rpmL_cmd = rpmL_cur - correction
            rpmR_cmd = rpmR_cur + correction

            # ===== ODOM ESTIMATION =====
            omegaL = (rpmL_cmd * 2.0 * math.pi) / 60.0
            omegaR = (rpmR_cmd * 2.0 * math.pi) / 60.0

            vL = omegaL * wheel_radius
            vR = omegaR * wheel_radius

            vx = 0.5 * (-vL + vR)
            wz = (vR + vL) / wheel_base

            # ===== publish wheel_twist =====
            msg = TwistWithCovarianceStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = node.frame_id

            msg.twist.twist.linear.x = float(vx)
            msg.twist.twist.angular.z = float(wz)

            cov = [0.0] * 36
            cov[0] = 0.02
            cov[35] = 0.05
            msg.twist.covariance = cov

            node.pub_twist_cov.publish(msg)

            # ===== READ DRIVER ERROR (2Hz) =====
            if int(time.time() * 10) % 5 == 0:
                errL = read_driver_error(ser, NODE_L)
                errR = read_driver_error(ser, NODE_R)

                if errL not in [None, 0]:
                    error_flag = True
                if errR not in [None, 0]:
                    error_flag = True

            # ===== DETECT CAN =====
            if time.time() - last_ok_time > ERROR_TIMEOUT:
                print("❌ Lost CAN / Driver!")
                error_flag = True

            # ===== AUTO STOP =====
            if error_flag:
                emergency_stop(ser)
                continue

            # ===== send =====
            set_speed_dec_from_rpm(ser, NODE_L, rpmL_cmd)
            set_speed_dec_from_rpm(ser, NODE_R, rpmR_cmd)

            print(f"L={rpmL_cmd:.1f} | R={rpmR_cmd:.1f}")

            # ===== timing =====
            next_t += dt
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()

    # except KeyboardInterrupt:
    #     print("Stopping with delay...")

    #     v = 0.3
    #     w = 0.0

    #     vL = v - w * wheel_base / 2
    #     vR = v + w * wheel_base / 2

    #     rpmL = (vL / (2*math.pi*wheel_radius)) * 60 * left_sign
    #     rpmR = (vR / (2*math.pi*wheel_radius)) * 60 * right_sign

    #     set_speed_dec_from_rpm(ser, NODE_L, rpmL)
    #     set_speed_dec_from_rpm(ser, NODE_R, rpmR)

    #     time.sleep(2.0)

    #     set_speed_dec_from_rpm(ser, NODE_L, 0)
    #     set_speed_dec_from_rpm(ser, NODE_R, 0)

    #     ser.close()
    #     print("Stopped.")
    except KeyboardInterrupt:
        print("Ctrl+C pressed")

    finally:
        print("🛑 SAFE STOP")

        try:
            # gửi 0 nhiều lần để đảm bảo driver nhận
            for _ in range(10):
                set_speed_dec_from_rpm(ser, NODE_L, 0)
                set_speed_dec_from_rpm(ser, NODE_R, 0)
                time.sleep(0.02)

        except:
            pass

        ser.close()
        print("Stopped safely.")
    

if __name__ == "__main__":
    main()