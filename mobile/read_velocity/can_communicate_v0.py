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

CMD_TIMEOUT = 1.0 #0.5      # nếu quá 0.3s ko có cmd_vel thì stop
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

def sdo_read_u16(ser, node, index, sub):
    """Đọc số nguyên 16-bit không dấu (16U) - Bạn đã có hàm này, 
    nhưng hãy đảm bảo dùng struct.unpack để chuẩn xác hơn"""
    sdo_send_read(ser, node, index, sub)
    t_end = time.time() + 0.02
    while time.time() < t_end:
        got = ws_read_one(ser, timeout=0.001)
        if not got: continue
        cid, d = got
        if cid != 0x580 + node or len(d) < 6: continue
        if d[1] != (index & 0xFF) or d[2] != ((index >> 8) & 0xFF) or d[3] != sub:
            continue
        return struct.unpack("<H", d[4:6])[0]
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

def sdo_read_i16(ser, node, index, sub):
    """Đọc số nguyên 16-bit có dấu (16S)"""
    sdo_send_read(ser, node, index, sub)
    t_end = time.time() + 0.02
    while time.time() < t_end:
        got = ws_read_one(ser, timeout=0.001)
        if not got: continue
        cid, d = got
        if cid != 0x580 + node or len(d) < 6: continue
        if d[1] != (index & 0xFF) or d[2] != ((index >> 8) & 0xFF) or d[3] != sub:
            continue
        # Giải mã 2 byte (d[4], d[5]) thành số nguyên 16-bit có dấu
        return struct.unpack("<h", d[4:6])[0]
    return None

def dec_to_arms(dec_val):
    if dec_val is None: return 0.0
    # Công thức đảo: Arms = (DEC * 33) / (1.414 * 2048)
    return (dec_val * 33.0) / (1.414 * 2048.0)

def read_error_detail(ser, node):
    err = sdo_read_u32(ser, node, 0x2601, 0x00)
    if err is None:
        print(f"[Node {node}] ❌ Cannot read error detail")
        return None

    print(f"[Node {node}] 🔴 Error detail: 0x{err:08X} ({err})")
    return err
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

    # --- BỔ SUNG DÒNG NÀY ---
    # Thiết lập gia tốc dừng khẩn cấp mượt (khoảng 50 DEC ~ 0.7 rps/s^2)
    # 0x605A:01 là 32U nên dùng sdo_write_i32
    sdo_write_i32(ser, node, 0x605A, 0x01, 50) 
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
# def set_acceleration(ser, node, dec_value):
#     # dec_value: giá trị DEC (ví dụ 150)
#     b = struct.pack("<I", dec_value)  # U32 little-endian

#     payload = bytes([
#         0x23,                   # ghi 4 byte
#         0x83, 0x60,             # index 0x6083
#         0x00,                   # subindex
#         b[0], b[1], b[2], b[3]
#     ])

    # ws_write8(ser, 0x600 + node, payload)

def set_scurve_dec_start(ser, node, rpm_value):
    """
    Set S-curve start point at deceleration
    Index: 0x5000
    Sub:   0x02
    Value: U32 (convert từ rpm)
    """

    dec_val = int(round(rpm_value * SCALE_DEC_PER_RPM))
    b = struct.pack("<I", dec_val)

    payload = bytes([
        0x23,                   # write 4 byte
        0x00, 0x50,             # index 0x5000
        0x02,                   # subindex
        b[0], b[1], b[2], b[3]
    ])

    ws_write8(ser, 0x600 + node, payload)

# class CmdVelSub(Node):
#     def __init__(self):
#         super().__init__('can_cmdvel_sub')
#         self.v = 0.0
#         self.w = 0.0
#         self.last = time.time()
#         self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

#     def cb(self, msg: Twist):
#         self.v = float(msg.linear.x)
#         self.w = float(msg.angular.z)
#         self.last = time.time()

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
    # ===== SET S-CURVE DEC START =====
    print("Setting S-curve dec start...")

    set_scurve_dec_start(ser, NODE_L, 100)   # 100 rpm (khuyến nghị)
    set_scurve_dec_start(ser, NODE_R, 100)
    time.sleep(0.2)

    
    

    # time.sleep(0.05)

     # ===== READ DRIVER PARAM =====
    print("==== DRIVER PARAM CHECK ====")



    estop_L = sdo_read_u8(ser, NODE_L, 0x605A, 0x11)
    estop_R = sdo_read_u8(ser, NODE_R, 0x605A, 0x11)

    val_L = sdo_read_u32(ser, NODE_L, 0x605A, 0x01)
    val_R = sdo_read_u32(ser, NODE_R, 0x605A, 0x01)

    DEC_L = sdo_read_u32(ser, NODE_L, 0x6084, 0x00)
    DEC_R = sdo_read_u32(ser, NODE_R, 0x6084, 0x00)

    DEC_START_L = sdo_read_u32(ser, NODE_L, 0x5000, 0x02)
    DEC_START_R = sdo_read_u32(ser, NODE_R, 0x5000, 0x02)

    print("==== NEW PARAM CHECK FROM IMAGE ====")
    
    for node in [NODE_L, NODE_R]:
        name = "LEFT" if node == NODE_L else "RIGHT"
        
        # 1. Đọc Output Current (0x60F6:08) - 16S
        curr_dec = sdo_read_i16(ser, node, 0x60F6, 0x08)
        curr_arms = dec_to_arms(curr_dec)
        
        # 2. Đọc Output Current Limit (0x6073:00) - 16U
        limit_dec = sdo_read_u16(ser, node, 0x6073, 0x00)
        limit_arms = dec_to_arms(limit_dec)
        
        # 3. Đọc Max Speed RPM (0x6080:00) - 16U
        max_speed = sdo_read_u16(ser, node, 0x6080, 0x00)
        
        print(f"[{name} Node {node}]")
        print(f"  -> Output Current: {curr_arms:.2f} Arms (DEC: {curr_dec})")
        print(f"  -> Current Limit : {limit_arms:.2f} Arms (DEC: {limit_dec})")
        print(f"  -> Max Speed     : {max_speed} rpm")

    print(f"E-STOP L: {estop_L} | R: {estop_R}")
    print(f"0x605A01 L: {val_L} | R: {val_R}")
    print(f"0x608400 L: {DEC_L} | R: {DEC_R}")
    print(f"S-curve DEC start L: {DEC_START_L} | R: {DEC_START_R}")

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
        last_dec_mode = None
        while True:
            rclpy.spin_once(cmd,timeout_sec=0.0)
            # nếu mất cmd_vel quá lâu thì stop
            # 1. Lấy mục tiêu từ ROS, nếu mất lệnh thì mục tiêu là 0.0
            current_time = time.time()
            print(f"Giá trị time.time() hiện tại: {current_time}")
            if time.time() - cmd.last > CMD_TIMEOUT:
                v = 0.0
                w = 0.0
            else:
                v = cmd.v
                w = cmd.w

            # 2. Bộ lọc Ramp (Gia tốc phần mềm)
            # 0.005 m/s mỗi 10ms tương đương 0.5 m/s^2 (Rất mượt cho PIN và Driver)
            # MAX_V_STEP = 0.005 
            # MAX_W_STEP = 0.02

            # if target_v > last_v_sent:
            #     v_send = min(target_v, last_v_sent + MAX_V_STEP)
            # else:
            #     v_send = max(target_v, last_v_sent - MAX_V_STEP)

            # if target_w > last_w_sent:
            #     w_send = min(target_w, last_w_sent + MAX_W_STEP)
            # else:
            #     w_send = max(target_w, last_w_sent - MAX_W_STEP)

            # # Cập nhật biến trạng thái để dùng cho vòng lặp sau
            # last_v_sent = v_send
            # last_w_sent = w_send

            # # 3. Tính toán Kinematics từ vận tốc đã được làm mượt
            # vL = v_send - w_send * wheel_base / 2.0
            # vR = v_send + w_send * wheel_base / 2.0

            # # Chuyển đổi sang RPM
            # rpmL = (vL / (2.0 * 3.1415926535 * wheel_radius)) * 60.0
            # rpmR = (vR / (2.0 * 3.1415926535 * wheel_radius)) * 60.0


            # diff-drive kinematics (Code cũ của bạn)

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

            if abs(lastL) < 0.01 and abs(lastR) < 0.01:
                errL = read_error_detail(ser, NODE_L)
                errR = read_error_detail(ser, NODE_R)
                # print(f"errL={errL:.3f} rpm | errR ={errR :.3f} rpm")

                if errL is not None and errR is not None:
                    print(f"errL=0x{errL:X} | errR=0x{errR:X}")
                else:
                    print(f"errL={errL} | errR={errR} (read fail)")
            # ===== wheel rpm -> robot twist (m/s, rad/s) =====
            # lastL,lastR đang là rpm (wheel rpm). đổi ra m/s của từng bánh:
            omegaL = (lastL * 2.0 * math.pi) / 60.0
            omegaR = (lastR * 2.0 * math.pi) / 60.0
            vL_mps = omegaL * wheel_radius
            vR_mps = omegaR * wheel_radius

            # diff-drive: vx, wz
            vx = 0.5 * (-vL_mps + vR_mps)
            wz = (vR_mps + vL_mps) / wheel_base

            # ===== AUTO DECELERATION CONTROL =====
            speed = abs(vx)

            if speed <= 0.3 and last_dec_mode != "LOW":
                set_deceleration(ser, NODE_L, 220)
                set_deceleration(ser, NODE_R, 220)
                last_dec_mode = "LOW"

            elif 0.3 < speed <= 0.5 and last_dec_mode != "MID":
                set_deceleration(ser, NODE_L, 220)
                set_deceleration(ser, NODE_R, 220)
                last_dec_mode = "MID"

            elif 0.5 < speed <= 1.0 and last_dec_mode != "HIGH":
                set_deceleration(ser, NODE_L, 220)
                set_deceleration(ser, NODE_R, 220)
                last_dec_mode = "HIGH"

            elif speed > 1.0 and last_dec_mode != "HH":
                set_deceleration(ser, NODE_L, 220)
                set_deceleration(ser, NODE_R, 220)
                last_dec_mode = "HH"

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

# def main():
#     # 1. Khởi tạo ROS 2 trước vòng lặp kết nối
#     rclpy.init()
#     cmd = CmdVelSub()
    
#     # Khởi tạo các biến trạng thái vận tốc (để bộ lọc Ramp hoạt động liên tục)
#     global last_v_sent, last_w_sent
#     last_v_sent = 0.0
#     last_w_sent = 0.0
#     last_dec_mode = None
#     lastL = 0.0
#     lastR = 0.0

#     print(f"Starting Robot Driver with Auto-Reconnect on {PORT}...")

#     # VÒNG LẶP KẾT NỐI (Bao bọc toàn bộ để tự reconnect)
#     while rclpy.ok():
#         ser = None
#         try:
#             # 2. Thực hiện kết nối Serial
#             ser = serial.Serial(PORT, baudrate=SER_BAUD, timeout=0.001)
#             time.sleep(0.5)
#             print(f"✅ Connected to {PORT}. Initializing Driver...")

#             # 3. Khởi tạo Driver (Sẽ chạy lại mỗi khi reconnect)
#             init_speed_mode(ser, NODE_L)
#             init_speed_mode(ser, NODE_R)
#             set_scurve_dec_start(ser, NODE_L, 100)
#             set_scurve_dec_start(ser, NODE_R, 100)
#             time.sleep(0.2)
#             print("🚀 Driver Ready!")

#             dt = 1.0 / SEND_HZ 
#             next_t = time.monotonic()

#             # VÒNG LẶP ĐIỀU KHIỂN CHÍNH (Chạy liên tục khi có kết nối)
#             while rclpy.ok():
#                 rclpy.spin_once(cmd, timeout_sec=0.0)

#                 # --- LOGIC BỘ LỌC RAMP (Bạn đã có) ---
#                 if time.time() - cmd.last > CMD_TIMEOUT:
#                     target_v, target_w = 0.0, 0.0
#                 else:
#                     target_v, target_w = cmd.v, cmd.w

#                 MAX_V_STEP, MAX_W_STEP = 0.005, 0.02
                
#                 if target_v > last_v_sent: v_send = min(target_v, last_v_sent + MAX_V_STEP)
#                 else: v_send = max(target_v, last_v_sent - MAX_V_STEP)

#                 if target_w > last_w_sent: w_send = min(target_w, last_w_sent + MAX_W_STEP)
#                 else: w_send = max(target_w, last_w_sent - MAX_W_STEP)

#                 last_v_sent, last_w_sent = v_send, w_send

#                 # Tính Kinematics và gửi lệnh CAN
#                 vL = (v_send - w_send * wheel_base / 2.0) * left_sign
#                 vR = (v_send + w_send * wheel_base / 2.0) * right_sign
                
#                 rpmL = max(-MAX_RPM, min(MAX_RPM, (vL / (2.0 * math.pi * wheel_radius)) * 60.0))
#                 rpmR = max(-MAX_RPM, min(MAX_RPM, (vR / (2.0 * math.pi * wheel_radius)) * 60.0))

#                 # Gửi lệnh - Nếu lỗi ở đây sẽ nhảy xuống khối 'except'
#                 set_speed_dec_from_rpm(ser, NODE_L, rpmL)
#                 time.sleep(0.001)
#                 set_speed_dec_from_rpm(ser, NODE_R, rpmR)

#                 # --- ĐỌC PHẢN HỒI VÀ PUBLISH ODOMETRY (Như cũ) ---
#                 sdo_send_read(ser, NODE_L, 0x60F9, 0x19)
#                 sdo_send_read(ser, NODE_R, 0x60F9, 0x19)
                
#                 # ... (Giữ nguyên đoạn xử lý try_take_speed_reply và publish EKF của bạn ở đây) ...
#                 # (Để ngắn gọn tôi không lặp lại toàn bộ logic publish odom, bạn cứ giữ nguyên)
                
#                 # Kiểm tra cổng còn mở không (đề phòng)
#                 if not ser.is_open:
#                     raise serial.SerialException("Port disconnected")

#                 # Điều khiển tần suất vòng lặp
#                 next_t += dt
#                 sleep_s = next_t - time.monotonic()
#                 if sleep_s > 0: time.sleep(sleep_s)
#                 else: next_t = time.monotonic()

#         except (serial.SerialException, OSError, BrokenPipeError) as e:
#             print(f"⚠️ Connection lost: {e}. Retrying in 2s...")
#             if ser: 
#                 try: ser.close()
#                 except: pass
#             time.sleep(2.0) # Đợi trước khi thử lại

#         except KeyboardInterrupt:
#             print("Stopping...")
#             break

#     # Kết thúc
#     if ser and ser.is_open:
#         set_speed_dec_from_rpm(ser, NODE_L, 0.0)
#         set_speed_dec_from_rpm(ser, NODE_R, 0.0)
#         ser.close()
#     rclpy.shutdown()

if __name__ == "__main__":
    main()
