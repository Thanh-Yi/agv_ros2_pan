#!/usr/bin/env python3
# hwt901b_modbus_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import struct 
import time
import math
import sys

# ---- Helper: CRC16 (Modbus) ----
def modbus_crc(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return struct.pack('<H', crc)  # little endian: lo, hi

# Build a Modbus read holding registers request
def build_read_request(slave: int, start_addr: int, count: int) -> bytes:
    pkt = bytes([slave, 0x03, (start_addr >> 8) & 0xFF, start_addr & 0xFF, (count >> 8) & 0xFF, count & 0xFF])
    return pkt + modbus_crc(pkt)

# Parse response for function 0x03: [slave][0x03][byte_count][data...][crc_lo][crc_hi]
def parse_modbus_response(resp: bytes):
    if len(resp) < 5:
        return None
    # validate CRC
    payload, crc = resp[:-2], resp[-2:]
    if modbus_crc(payload) != crc:
        return None
    if payload[1] != 0x03:
        return None
    byte_count = payload[2]
    data = payload[3:3+byte_count]
    return data

def regs_to_int16_list(data: bytes):
    # Modbus registers big-endian: high byte first
    vals = []
    for i in range(0, len(data), 2):
        hi = data[i]; lo = data[i+1]
        val = struct.unpack('>h', bytes([hi, lo]))[0]
        vals.append(val)
    return vals

class HWT901BModbusNode(Node):
    def __init__(self):
        super().__init__('hwt901b_modbus')
        self.declare_parameter('port', '/dev/ch34x_imu')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('slave_id', 0x50)
        self.declare_parameter('poll_hz', 50.0)
        # scales (can tune)
        self.declare_parameter('acc_scale_g', 16.0)    # ±16g
        self.declare_parameter('gyro_scale_dps', 2000.0) # ±2000 dps

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.slave = self.get_parameter('slave_id').get_parameter_value().integer_value
        self.poll_hz = float(self.get_parameter('poll_hz').get_parameter_value().double_value)
        self.acc_scale_g = float(self.get_parameter('acc_scale_g').get_parameter_value().double_value)
        self.gyro_scale_dps = float(self.get_parameter('gyro_scale_dps').get_parameter_value().double_value)

        self.frame_id = 'imu'
        self.get_logger().info(f"HWT901B-485 Modbus node init: {port}@{baud} slave=0x{self.slave:02X} poll={self.poll_hz}Hz")

        try:
            self.ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
        except Exception as e:
            self.get_logger().error(f"Cannot open serial port {port}: {e}")
            raise

        # Try to enable RS485 mode if available
        try:
            from serial.rs485 import RS485Settings
            self.ser.rs485_mode = RS485Settings()  # default toggles RTS
            self.get_logger().info("Enabled serial.rs485_mode() on adapter")
            self.rs485_available = True
        except Exception:
            self.get_logger().warning("rs485_mode not set (adapter may still auto toggle). Will toggle RTS manually if needed.")
            self.rs485_available = False

        # Publisher
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # prepare requests (addresses per manufacturer docs; adjust if vendor doc differs)
        # Typical HWT901B-485 register map (example from vendor doc). Adjust start addresses if your module differs.
        self.addr_acc = 0x0034   # read 3 registers -> ax ay az
        self.addr_gyro = 0x0037  # read 3 registers -> gx gy gz
        self.addr_angle = 0x003D # read 3 registers -> roll pitch yaw

        self.req_acc = build_read_request(self.slave, self.addr_acc, 3)
        self.req_gyro = build_read_request(self.slave, self.addr_gyro, 3)
        self.req_angle = build_read_request(self.slave, self.addr_angle, 3)

        # timer
        period = 1.0 / max(1.0, self.poll_hz)
        self.timer = self.create_timer(period, self.poll_once)

        # last values
        self.acc = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.orientation_q = (0.0, 0.0, 0.0, 1.0)

    def send_request(self, req: bytes) -> bytes:
        # if RS485 mode not available, toggle RTS manually: assert before write, deassert after short wait
        if not self.rs485_available:
            try:
                # set RTS high to enable DE on many adapters
                self.ser.setRTS(True)
                time.sleep(0.001)
            except Exception:
                pass
        self.ser.write(req)
        self.ser.flush()
        # give device a little time to respond
        time.sleep(0.02)
        # read available bytes (non-blocking due to timeout)
        resp = bytearray()
        # read a few chunks until timeout
        try:
            while True:
                chunk = self.ser.read(256)
                if not chunk:
                    break
                resp.extend(chunk)
                # small pause to allow rest of frame
                time.sleep(0.005)
        except Exception as e:
            self.get_logger().warning(f"Serial read exception: {e}")
        if not self.rs485_available:
            try:
                self.ser.setRTS(False)
            except Exception:
                pass
        return bytes(resp)

    def poll_once(self):
        # request acc
        resp = self.send_request(self.req_acc)
        data = parse_modbus_response(resp) if resp else None
        if data:
            vals = regs_to_int16_list(data)
            if len(vals) >= 3:
                ax = vals[0] * (self.acc_scale_g * 9.80665) / 32768.0
                ay = vals[1] * (self.acc_scale_g * 9.80665) / 32768.0
                az = vals[2] * (self.acc_scale_g * 9.80665) / 32768.0
                self.acc = (ax, ay, az)

        # request gyro
        resp = self.send_request(self.req_gyro)
        data = parse_modbus_response(resp) if resp else None
        if data:
            vals = regs_to_int16_list(data)
            if len(vals) >= 3:
                # convert degree/s -> rad/s
                scale = (self.gyro_scale_dps * math.pi / 180.0) / 32768.0
                gx = vals[0] * scale
                gy = vals[1] * scale
                gz = vals[2] * scale
                self.gyro = (gx, gy, gz)

        # request angle (yaw/pitch/roll typical)
        resp = self.send_request(self.req_angle)
        data = parse_modbus_response(resp) if resp else None
        if data:
            vals = regs_to_int16_list(data)
            if len(vals) >= 3:
                # vendor docs: roll/pitch/yaw stored as int16 where value/32768*pi => radians
                roll = vals[0] / 32768.0 * math.pi
                pitch = vals[1] / 32768.0 * math.pi
                yaw = vals[2] / 32768.0 * math.pi
                # simple yaw->quat (assuming roll=pitch=0 if not provided)
                qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
                self.orientation_q = (qx, qy, qz, qw)

        # publish
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.linear_acceleration.x = float(self.acc[0])
        msg.linear_acceleration.y = float(self.acc[1])
        msg.linear_acceleration.z = float(self.acc[2])

        msg.angular_velocity.x = float(self.gyro[0])
        msg.angular_velocity.y = float(self.gyro[1])
        msg.angular_velocity.z = float(self.gyro[2])

        qx, qy, qz, qw = self.orientation_q
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.orientation.w = float(qw)

        self.pub.publish(msg)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
        return q

def main(args=None):
    rclpy.init(args=args)
    node = HWT901BModbusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
