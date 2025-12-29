#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import serial, time, math
from threading import Thread, Lock

def u16_le(buf, off):
    return buf[off] | (buf[off+1] << 8)

class LDS50CR(Node):
    def __init__(self):
        super().__init__('lds50cr')

        self.declare_parameter('port', '/dev/cp210x_lidar')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('min_range', 0.4)
        self.declare_parameter('max_range', 40.0)
        self.declare_parameter('wrap_threshold_deg', 10.0)

        p = self.get_parameter
        self.port = p('port').value
        self.baud = int(p('baud').value)
        self.laser_frame = p('laser_frame').value
        self.min_range = float(p('min_range').value)
        self.max_range = float(p('max_range').value)
        self.wrap_threshold = float(p('wrap_threshold_deg').value)

        self.pub = self.create_publisher(LaserScan, 'scan_raw', 10)

        self.ser = serial.Serial(self.port, self.baud, timeout=0.2)

        self.buf = bytearray()
        self.lock = Lock()
        self.running = True
        Thread(target=self._reader, daemon=True).start()

        self.pkts = []
        self.last_start = None

        self.timer = self.create_timer(0.03, self._tick)

    def _reader(self):
        while self.running and rclpy.ok():
            data = self.ser.read(8192)
            if data:
                with self.lock:
                    self.buf.extend(data)

    def _extract(self):
        out = []
        with self.lock:
            data = bytes(self.buf)

        pos = 0
        while True:
            h = data.find(b'\xCF\xFA', pos)
            if h < 0 or h + 8 > len(data):
                break
            n = u16_le(data, h+2)
            need = 8 + n*3
            if h + need > len(data):
                break
            out.append(data[h:h+need])
            pos = h + need

        if pos > 0:
            with self.lock:
                del self.buf[:pos]
        return out

    def _tick(self):
        raws = self._extract()
        for raw in raws:
            start = u16_le(raw, 4) * 0.1

            if self.last_start is not None:
                if start < self.last_start - self.wrap_threshold:
                    self._publish()
                    self.pkts = []

            self.pkts.append(raw)
            self.last_start = start

    def _publish(self):
        if not self.pkts:
            return

        NUM = 1800
        ranges = [float('inf')] * NUM
        intens = [0.0] * NUM
        used = [False] * NUM   # <<< FIX QUAN TRỌNG NHẤT

        for raw in self.pkts:
            n = u16_le(raw, 2)
            start = u16_le(raw, 4) * 0.1
            sector = u16_le(raw, 6) * 0.1
            step = sector / max(1, n-1)

            off = 8
            for i in range(n):
                r = (raw[off+1] | (raw[off+2] << 8)) / 1000.0
                inten = raw[off]
                off += 3

                if r < self.min_range or r > self.max_range:
                    continue

                angle = (start + i*step) % 360.0
                idx = int(angle / 360.0 * NUM)

                if used[idx]:
                    continue    # <<< KHÔNG GHI LẠI

                ranges[idx] = r
                intens[idx] = float(inten)
                used[idx] = True

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.laser_frame
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = (msg.angle_max - msg.angle_min) / NUM
        msg.range_min = self.min_range
        msg.range_max = self.max_range
        msg.ranges = ranges
        msg.intensities = intens

        self.pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.ser.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = LDS50CR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
