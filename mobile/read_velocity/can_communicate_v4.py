#!/usr/bin/env python3
"""
can_communicate.py — ROS2 CANopen motor driver via Waveshare USB-CAN-A.

Hardware: Jetson → USB-CAN-A (serial 2 Mbps) → CAN bus → L2DB4830 driver
  Node 1 = left motor, Node 2 = right motor (differential drive)

ROS2 topics:
  Subscribe:
    /cmd_vel                    geometry_msgs/Twist
    /motor_driver/read_param    std_msgs/String  JSON {node, index, subindex, fmt}
    /motor_driver/write_param   std_msgs/String  JSON {node, index, subindex, fmt, value}
  Publish:
    /wheel_twist                geometry_msgs/TwistWithCovarianceStamped
    /odom                       nav_msgs/Odometry
    /motor_driver/status        std_msgs/String  JSON (5 Hz)
    /motor_driver/param_result  std_msgs/String  JSON
"""

import json
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import serial  # pyserial — installed on robot (pip install pyserial)

# ── Hardware ──────────────────────────────────────────────────────────────────
PORT               = "/dev/ch34x_can"
SER_BAUD           = 2_000_000
SER_TIMEOUT        = 0.05          # s — serial read timeout
RECONNECT_INTERVAL = 3.0           # s — retry delay when disconnected

# ── Robot geometry ────────────────────────────────────────────────────────────
WHEEL_RADIUS = 0.08437 # m  (calibrated: 9.0m actual / 9.0667m odom × 0.085)
WHEEL_BASE   = 0.35    # m
LEFT_SIGN    = -1.0    # flip left motor direction (mounting orientation)
RIGHT_SIGN   =  1.0

# ── CANopen ───────────────────────────────────────────────────────────────────
NODE_L = 1
NODE_R = 2

# L2DB4830 series
ENCODER_RESOLUTION  = 4096
I_MAX               = 33.0         # A — driver max output current

# DEC ↔ RPM: [DEC] = [rpm] * 512 * Resolution / 1875
SCALE_DEC_PER_RPM   = 512.0 * ENCODER_RESOLUTION / 1875.0    # = 1118.48…

# DEC ↔ rps/s: [DEC] = [rps/s] * 256 * Resolution / 15625
SCALE_DEC_PER_RPS_S = 256.0 * ENCODER_RESOLUTION / 15625.0   # =  67.11…

# ── Control parameters ────────────────────────────────────────────────────────
MAX_RPM           = 200.0
CMD_TIMEOUT       = 1.0    # s — send zero if no new /cmd_vel
CTRL_HZ           = 100    # Hz — velocity send rate
STATUS_HZ         = 5      # Hz — /motor_driver/status publish rate

# Default acceleration / deceleration (DEC units, set at init)
DEFAULT_ACCEL_DEC = 224    # ≈ 200 rpm/s
DEFAULT_DECEL_DEC = 224    # ≈ 200 rpm/s
DEFAULT_EMERG_DEC = 560    # ≈ 500 rpm/s — emergency stop

# Adaptive deceleration bands (DEC units) keyed by commanded RPM threshold
# Entry: (max_rpm_threshold, decel_dec)
DECEL_BANDS = [
    (5.0,   112),   # < 5 rpm  → 100 rpm/s (gentle)
    (30.0,  224),   # < 30 rpm → 200 rpm/s (normal)
    (100.0, 448),   # < 100rpm → 400 rpm/s (strong)
    (999.0, 784),   # ≥ 100rpm → 700 rpm/s (aggressive)
]


# ── CANopen / Waveshare USB-CAN-A helpers ─────────────────────────────────────

def _can_frame(cob_id: int, data: bytes) -> bytes:
    """Build Waveshare USB-CAN-A TX frame for a standard 8-byte CAN data frame."""
    payload = (bytes(data) + b'\x00' * 8)[:8]
    # Waveshare USB-CAN-A: standard data frame INFO = 0xC0 | DLC
    info = 0xC0 | (len(payload) & 0x0F)
    return bytes([0xAA, info, cob_id & 0xFF, (cob_id >> 8) & 0xFF]) + payload + bytes([0x55])


def _scan_frame(buf: bytearray):
    """
    Scan buf for the first complete Waveshare frame.
    Returns (cob_id, data_bytes, consumed_bytes) or None if incomplete/not found.
    """
    i = 0
    while i < len(buf):
        if buf[i] != 0xAA:
            i += 1
            continue
        if i + 4 > len(buf):
            break                       # need more data
        dlc   = buf[i + 1] & 0x0F
        total = 5 + dlc                 # AA + info + id_lo + id_hi + data + 55
        if i + total > len(buf):
            break                       # incomplete frame
        if buf[i + total - 1] != 0x55:
            i += 1                      # not a valid frame start, keep searching
            continue
        cob_id = buf[i + 2] | (buf[i + 3] << 8)
        data   = bytes(buf[i + 4 : i + 4 + dlc])
        return cob_id, data, i + total
    # Discard unreachable preamble bytes before i
    return None if i == 0 else (None, None, i)


def sdo_write(ser: serial.Serial, node: int, index: int, subindex: int,
              fmt: str, value) -> bool:
    """CANopen SDO expedited write. Returns True on ACK, False on error/timeout."""
    cob_tx = 0x600 + node
    cob_rx = 0x580 + node
    sz  = {'u8': 1, 'i8': 1, 'u16': 2, 'i16': 2, 'u32': 4, 'i32': 4}[fmt]
    cmd = {1: 0x2F, 2: 0x2B, 4: 0x23}[sz]
    pk  = {'u8': '<B', 'i8': '<b', 'u16': '<H', 'i16': '<h', 'u32': '<I', 'i32': '<i'}[fmt]
    val_bytes = (struct.pack(pk, value) + b'\x00\x00\x00\x00')[:4]
    req  = struct.pack('<BHB', cmd, index, subindex) + val_bytes   # 4 + 4 = 8 bytes

    ser.reset_input_buffer()
    ser.write(_can_frame(cob_tx, req))

    rx_buf   = bytearray()
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            rx_buf.extend(chunk)
        result = _scan_frame(rx_buf)
        if result is None:
            continue
        cid, d, consumed = result
        del rx_buf[:consumed]
        if cid is None:
            continue
        if cid == cob_rx and d and len(d) >= 4:
            if d[1] == (index & 0xFF) and d[2] == (index >> 8) and d[3] == subindex:
                return d[0] == 0x60   # 0x60 = success, 0x80 = abort
    return False


def sdo_read(ser: serial.Serial, node: int, index: int, subindex: int, fmt: str):
    """CANopen SDO expedited read. Returns value or None on error/timeout."""
    cob_tx = 0x600 + node
    cob_rx = 0x580 + node
    pk     = {'u8': '<B', 'i8': '<b', 'u16': '<H', 'i16': '<h', 'u32': '<I', 'i32': '<i'}[fmt]
    pk_sz  = struct.calcsize(pk)
    req    = struct.pack('<BHB', 0x40, index, subindex) + b'\x00\x00\x00\x00'  # 4 + 4 = 8 bytes

    ser.reset_input_buffer()
    ser.write(_can_frame(cob_tx, req))

    rx_buf   = bytearray()
    deadline = time.monotonic() + 0.15
    while time.monotonic() < deadline:
        chunk = ser.read(max(1, ser.in_waiting))
        if chunk:
            rx_buf.extend(chunk)
        result = _scan_frame(rx_buf)
        if result is None:
            continue
        cid, d, consumed = result
        del rx_buf[:consumed]
        if cid is None:
            continue
        if cid == cob_rx and d and len(d) >= 8:
            if d[1] == (index & 0xFF) and d[2] == (index >> 8) and d[3] == subindex:
                if d[0] == 0x80:
                    return None         # abort response
                payload = bytes(d[4 : 4 + pk_sz]).ljust(pk_sz, b'\x00')
                return struct.unpack(pk, payload)[0]
    return None


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class CanMotorDriver(Node):

    def __init__(self):
        super().__init__('can_motor_driver')

        # Serial connection (guarded by _ser_lock)
        self._ser      : serial.Serial | None = None
        self._ser_lock    = threading.Lock()   # serializes serial I/O
        self._connect_lock = threading.Lock()  # prevents concurrent _connect() calls
        self._connected = False
        self._motors_initialized = False  # True sau khi _init_motors() thành công

        # Resilience counters — require N consecutive failures before declaring LOST/offline
        self._ctrl_fail_count    = 0  # consecutive SerialException in control loop
        self._driver_offline_cnt = 0  # consecutive status cycles with driver_ok=False
        self._last_good_ctrl     = time.monotonic()  # last time speed SDO reads succeeded

        # Status word + diagnostic cache — populated by control loop, read by status loop
        # (status loop không đụng serial để tránh block control loop trong single-threaded executor)
        self._ctrl_cycle_cnt = 0
        self._cached_sw_l    = None
        self._cached_sw_r    = None
        self._cached_rpm     = {'left': None, 'right': None}
        self._last_diag      = {'left': None, 'right': None}

        # Velocity command (guarded by _cmd_lock)
        self._cmd_lock      = threading.Lock()
        self._last_cmd_time = 0.0
        self._target_l_rpm  = 0.0
        self._target_r_rpm  = 0.0
        self._prev_decel    = -1      # track last decel sent to avoid redundant writes

        # Odometry integration
        self._odom_x     = 0.0
        self._odom_y     = 0.0
        self._odom_theta = 0.0
        self._odom_last  = time.monotonic()

        # Publishers
        self._twist_pub  = self.create_publisher(TwistWithCovarianceStamped, '/wheel_twist', 10)
        self._odom_pub   = self.create_publisher(Odometry, '/odom', 10)
        self._status_pub = self.create_publisher(String, '/motor_driver/status', 10)
        self._param_pub  = self.create_publisher(String, '/motor_driver/param_result', 10)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(String, '/motor_driver/read_param',
                                 self._on_read_param, 10)
        self.create_subscription(String, '/motor_driver/write_param',
                                 self._on_write_param, 10)
        self.create_subscription(String, '/motor_driver/reinit',
                                 self._on_reinit, 10)

        # Timers
        self.create_timer(1.0 / CTRL_HZ,   self._control_loop)
        self.create_timer(1.0 / STATUS_HZ,  self._status_loop)
        self.create_timer(RECONNECT_INTERVAL, self._try_reconnect)

        # Initial connection
        self._connect()

    # ── Connection management ─────────────────────────────────────────────────

    def _connect(self):
        if not self._connect_lock.acquire(blocking=False):
            self.get_logger().info('CAN connect already in progress, skipping')
            return
        try:
            # Step 1: close old port cleanly BEFORE opening a new one
            with self._ser_lock:
                old = self._ser
                self._ser = None
            if old:
                try:
                    old.close()
                except Exception:
                    pass
                time.sleep(0.4)  # let OS release the port before reopening

            # Step 2: open new port
            ser = serial.Serial(PORT, SER_BAUD, timeout=SER_TIMEOUT)
            with self._ser_lock:
                self._ser = ser

            # Step 3: init motors (connected stays False until done)
            self._init_motors()
            self._motors_initialized = True
            self._connected = True
            # reset counters so fresh state after reconnect
            self._ctrl_fail_count    = 0
            self._driver_offline_cnt = 0
            self._ctrl_cycle_cnt     = 0
            self._cached_sw_l        = None
            self._cached_sw_r        = None
            self._last_good_ctrl     = time.monotonic()
            self.get_logger().info(f'CAN connected on {PORT} @ {SER_BAUD}')
        except Exception as exc:
            self._connected = False
            self.get_logger().warn(f'CAN connect failed: {exc}')
        finally:
            self._connect_lock.release()

    def _try_reconnect(self):
        if not self._connected:
            self.get_logger().info('Reconnecting CAN...')
            self._connect()

    def _on_reinit(self, _msg: String):
        """Re-initialize motor driver after E-stop restore or driver power cycle."""
        if not self._connected or self._ser is None:
            self.get_logger().info('Reinit: CAN lost, triggering reconnect')
            threading.Thread(target=self._connect, daemon=True).start()
            return
        self.get_logger().info('Reinit motors requested from web UI')
        try:
            self._init_motors()
            self._motors_initialized = True
            self.get_logger().info('Motor reinit OK')
        except Exception as exc:
            self.get_logger().error(f'Motor reinit failed: {exc}')

    def _init_motors(self):
        for node in (NODE_L, NODE_R):
            self._init_speed_mode(node)

    def _init_speed_mode(self, node: int):
        ser = self._ser
        # 1. Clear any fault
        sdo_write(ser, node, 0x6040, 0x00, 'u16', 0x86)
        time.sleep(0.05)
        # 2. Set default acceleration / deceleration
        sdo_write(ser, node, 0x6083, 0x00, 'u32', DEFAULT_ACCEL_DEC)
        sdo_write(ser, node, 0x6084, 0x00, 'u32', DEFAULT_DECEL_DEC)
        sdo_write(ser, node, 0x605A, 0x01, 'u32', DEFAULT_EMERG_DEC)
        # 3. Speed mode with Acc/Dec (mode 3)
        sdo_write(ser, node, 0x6060, 0x00, 'i8', 3)
        time.sleep(0.02)
        # 4. Enable drive
        sdo_write(ser, node, 0x6040, 0x00, 'u16', 0x0F)
        time.sleep(0.02)

    # ── Velocity command ──────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist):
        """Handles geometry_msgs/Twist on /cmd_vel."""
        v = msg.linear.x    # m/s
        w = msg.angular.z   # rad/s
        # Differential drive kinematics → wheel speed [m/s]
        v_l = v - w * WHEEL_BASE / 2.0
        v_r = v + w * WHEEL_BASE / 2.0
        # Convert m/s → RPM
        rpm_l = v_l / WHEEL_RADIUS * 60.0 / (2.0 * math.pi)
        rpm_r = v_r / WHEEL_RADIUS * 60.0 / (2.0 * math.pi)
        rpm_l = max(-MAX_RPM, min(MAX_RPM, rpm_l))
        rpm_r = max(-MAX_RPM, min(MAX_RPM, rpm_r))
        with self._cmd_lock:
            self._target_l_rpm  = rpm_l
            self._target_r_rpm  = rpm_r
            self._last_cmd_time = time.monotonic()

    # ── Control loop (100 Hz) ─────────────────────────────────────────────────

    def _control_loop(self):
        if not self._connected or self._ser is None:
            return

        with self._cmd_lock:
            timed_out = (time.monotonic() - self._last_cmd_time) > CMD_TIMEOUT
            rpm_l = 0.0 if timed_out else self._target_l_rpm
            rpm_r = 0.0 if timed_out else self._target_r_rpm

        # Adaptive deceleration (only write when band changes)
        abs_max = max(abs(rpm_l), abs(rpm_r))
        new_decel = DEFAULT_DECEL_DEC
        for threshold, dec in DECEL_BANDS:
            if abs_max < threshold:
                new_decel = dec
                break

        # Piggyback slow reads inside the same lock — no separate status-loop serial access
        self._ctrl_cycle_cnt += 1
        read_sw   = (self._ctrl_cycle_cnt % 20  == 0)   # 5 Hz
        read_diag = (self._ctrl_cycle_cnt % 200 == 0)   # 0.5 Hz

        try:
            with self._ser_lock:
                if new_decel != self._prev_decel:
                    sdo_write(self._ser, NODE_L, 0x6084, 0x00, 'u32', new_decel)
                    sdo_write(self._ser, NODE_R, 0x6084, 0x00, 'u32', new_decel)
                    self._prev_decel = new_decel

                dec_l = int(rpm_l * LEFT_SIGN  * SCALE_DEC_PER_RPM)
                dec_r = int(rpm_r * RIGHT_SIGN * SCALE_DEC_PER_RPM)
                sdo_write(self._ser, NODE_L, 0x60FF, 0x00, 'i32', dec_l)
                sdo_write(self._ser, NODE_R, 0x60FF, 0x00, 'i32', dec_r)

                # Read actual speeds (0x60F9:0x19 → 0.001 rpm units, i32)
                raw_l = sdo_read(self._ser, NODE_L, 0x60F9, 0x19, 'i32')
                raw_r = sdo_read(self._ser, NODE_R, 0x60F9, 0x19, 'i32')

                # Chỉ piggyback reads khi speed reads thành công — tránh thêm 12 × 150ms
                # blocking vào executor khi driver không phản hồi (SDO None ≠ SerialException)
                if raw_l is not None and raw_r is not None:
                    self._last_good_ctrl = time.monotonic()

                    if read_sw:
                        self._cached_sw_l = sdo_read(self._ser, NODE_L, 0x6041, 0x00, 'u16')
                        self._cached_sw_r = sdo_read(self._ser, NODE_R, 0x6041, 0x00, 'u16')

                    if read_diag:
                        diag_new = {}
                        for lbl, nd in (('left', NODE_L), ('right', NODE_R)):
                            err_code = sdo_read(self._ser, nd, 0x2601, 0x00, 'u16')
                            voltage  = sdo_read(self._ser, nd, 0x60F7, 0x12, 'i16')
                            temp     = sdo_read(self._ser, nd, 0x60F7, 0x0B, 'i16')
                            iq_dec   = sdo_read(self._ser, nd, 0x6078, 0x00, 'i16')
                            diag_new[lbl] = {
                                'error_code':   err_code,
                                'voltage_v':    voltage,
                                'temp_c':       temp,
                                'current_arms': round(abs(iq_dec) * I_MAX / (1.414 * 2048), 3)
                                                if iq_dec is not None else None,
                            }
                        self._last_diag = diag_new

        except serial.SerialException as exc:
            self._ctrl_fail_count += 1
            if self._ctrl_fail_count >= 15:  # ~150ms consecutive errors at 100Hz
                self.get_logger().warn(
                    f'Serial error ×{self._ctrl_fail_count} in control loop: {exc}')
                self._connected = False
                self._ctrl_fail_count = 0
            return

        self._ctrl_fail_count = 0  # reset on successful serial exchange

        if raw_l is None or raw_r is None:
            # SDO timeout không raise SerialException — dùng thời gian để detect driver offline
            if time.monotonic() - self._last_good_ctrl > 3.0:  # 3s liên tục không nhận được data
                self.get_logger().warn('Speed SDO timeout >3s — declaring CAN offline')
                self._connected = False
                self._last_good_ctrl = time.monotonic()
            return

        actual_rpm_l = (raw_l * 0.001) * LEFT_SIGN
        actual_rpm_r = (raw_r * 0.001) * RIGHT_SIGN

        self._cached_rpm['left']  = actual_rpm_l
        self._cached_rpm['right'] = actual_rpm_r

        self._publish_wheel_twist(actual_rpm_l, actual_rpm_r)
        self._update_odometry(actual_rpm_l, actual_rpm_r)

    # ── Wheel twist publisher ─────────────────────────────────────────────────

    def _publish_wheel_twist(self, rpm_l: float, rpm_r: float):
        v_l = rpm_l * 2.0 * math.pi / 60.0 * WHEEL_RADIUS
        v_r = rpm_r * 2.0 * math.pi / 60.0 * WHEEL_RADIUS
        v   = (v_l + v_r) / 2.0
        w   = (v_r - v_l) / WHEEL_BASE

        msg = TwistWithCovarianceStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.twist.linear.x  = v
        msg.twist.twist.angular.z = w
        cov = [0.0] * 36
        cov[0]  = 1e-3   # vx variance (encoder tốt cho tốc độ tiến)
        cov[35] = 1e-3   # wz variance
        msg.twist.covariance = cov
        self._twist_pub.publish(msg)

    # ── Odometry integration ──────────────────────────────────────────────────

    def _update_odometry(self, rpm_l: float, rpm_r: float):
        now = time.monotonic()
        dt  = now - self._odom_last
        self._odom_last = now
        if dt <= 0.0 or dt > 0.5:
            return

        v_l = rpm_l * 2.0 * math.pi / 60.0 * WHEEL_RADIUS
        v_r = rpm_r * 2.0 * math.pi / 60.0 * WHEEL_RADIUS
        v   = (v_l + v_r) / 2.0
        w   = (v_r - v_l) / WHEEL_BASE

        self._odom_theta += w * dt
        self._odom_x     += v * math.cos(self._odom_theta) * dt
        self._odom_y     += v * math.sin(self._odom_theta) * dt

        qz = math.sin(self._odom_theta / 2.0)
        qw = math.cos(self._odom_theta / 2.0)
        stamp = self.get_clock().now().to_msg()

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp             = stamp
        odom.header.frame_id          = 'odom'
        odom.child_frame_id           = 'base_link'
        odom.pose.pose.position.x     = self._odom_x
        odom.pose.pose.position.y     = self._odom_y
        odom.pose.pose.orientation.z  = qz
        odom.pose.pose.orientation.w  = qw
        odom.twist.twist.linear.x     = v
        odom.twist.twist.angular.z    = w
        # Pose covariance (diagonal, indices 0 and 35)
        pc = [0.0] * 36
        pc[0]  = 1e-3
        pc[7]  = 1e-3
        pc[35] = 1e-3
        odom.pose.covariance = pc
        tc = [0.0] * 36
        tc[0]  = 1e-3
        tc[35] = 1e-3
        odom.twist.covariance = tc
        self._odom_pub.publish(odom)

    # ── Status loop (5 Hz) ────────────────────────────────────────────────────
    #
    # KHÔNG đọc serial ở đây. Toàn bộ SDO reads được "piggyback" vào control loop
    # (chạy cùng _ser_lock). Status loop chỉ đọc cache → không block control loop.
    #
    # Cache được cập nhật bởi control loop:
    #   _cached_sw_l/r  — status_word, mỗi 20 chu kỳ (5 Hz)
    #   _cached_rpm     — actual RPM, mỗi chu kỳ (100 Hz)
    #   _last_diag      — voltage/temp/current/error_code, mỗi 200 chu kỳ (0.5 Hz)

    def _status_loop(self):
        if not self._connected or self._ser is None:
            payload = json.dumps({'connected': False, 'driver_ok': False,
                                  'driver_enabled': False, 'left': None, 'right': None})
            msg = String(); msg.data = payload
            self._status_pub.publish(msg)
            return

        # Dùng cache từ control loop — không đụng serial
        sw_l = self._cached_sw_l
        sw_r = self._cached_sw_r
        driver_ok = (sw_l is not None) and (sw_r is not None)

        if not driver_ok:
            self._driver_offline_cnt += 1
            if self._driver_offline_cnt >= 15:  # 15 × 200ms = 3s liên tục offline
                self._motors_initialized = False
        else:
            self._driver_offline_cnt = 0

        driver_enabled = driver_ok and self._motors_initialized

        def _node_payload(label, sw):
            d = (self._last_diag.get(label) or {}).copy()
            d['status_word'] = sw
            d['fault']       = bool(sw & 0x08) if sw is not None else None
            d['speed_rpm']   = self._cached_rpm.get(label)
            return d if sw is not None else None

        payload = json.dumps({
            'connected':      self._connected,
            'driver_ok':      driver_ok,
            'driver_enabled': driver_enabled,
            'left':           _node_payload('left',  sw_l),
            'right':          _node_payload('right', sw_r),
        })
        msg = String(); msg.data = payload
        self._status_pub.publish(msg)

    # ── Web UI: parameter read / write ────────────────────────────────────────

    def _on_read_param(self, msg: String):
        """JSON in: {node, index (hex str or int), subindex (hex str or int), fmt}"""
        try:
            req     = json.loads(msg.data)
            node    = int(req['node'])
            index   = int(req['index'], 16) if isinstance(req['index'], str) else int(req['index'])
            subidx  = int(req['subindex'], 16) if isinstance(req['subindex'], str) else int(req['subindex'])
            fmt     = req.get('fmt', 'u32')

            if not self._connected or self._ser is None:
                raise RuntimeError('Not connected')

            with self._ser_lock:
                value = sdo_read(self._ser, node, index, subidx, fmt)

            result = json.dumps({
                'op':      'read',
                'node':     node,
                'index':    hex(index),
                'subindex': hex(subidx),
                'fmt':      fmt,
                'value':    value,
                'success':  value is not None,
            })
        except Exception as exc:
            result = json.dumps({'op': 'read', 'success': False, 'error': str(exc)})

        out = String(); out.data = result
        self._param_pub.publish(out)

    def _on_write_param(self, msg: String):
        """JSON in: {node, index, subindex, fmt, value}"""
        try:
            req    = json.loads(msg.data)
            node   = int(req['node'])
            index  = int(req['index'], 16) if isinstance(req['index'], str) else int(req['index'])
            subidx = int(req['subindex'], 16) if isinstance(req['subindex'], str) else int(req['subindex'])
            fmt    = req.get('fmt', 'u32')
            value  = int(req['value'])

            if not self._connected or self._ser is None:
                raise RuntimeError('Not connected')

            with self._ser_lock:
                ok = sdo_write(self._ser, node, index, subidx, fmt, value)

            result = json.dumps({
                'op':      'write',
                'node':     node,
                'index':    hex(index),
                'subindex': hex(subidx),
                'fmt':      fmt,
                'value':    value,
                'success':  ok,
            })
        except Exception as exc:
            result = json.dumps({'op': 'write', 'success': False, 'error': str(exc)})

        out = String(); out.data = result
        self._param_pub.publish(out)

    def destroy_node(self):
        # Send zero to both motors before shutdown
        if self._connected and self._ser:
            try:
                with self._ser_lock:
                    sdo_write(self._ser, NODE_L, 0x60FF, 0x00, 'i32', 0)
                    sdo_write(self._ser, NODE_R, 0x60FF, 0x00, 'i32', 0)
                    sdo_write(self._ser, NODE_L, 0x6040, 0x00, 'u16', 0x06)
                    sdo_write(self._ser, NODE_R, 0x6040, 0x00, 'u16', 0x06)
                    self._ser.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CanMotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
