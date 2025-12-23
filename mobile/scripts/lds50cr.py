#!/usr/bin/env python3
"""
LDS-50C-R ROS2 driver — final per-sector → per-rotation publisher
- Packet format (observed): CF FA | N(u16 LE) | start(u16 LE, 0.1deg) | sector(u16 LE, 0.1deg) | N * (I(1) Dlo(1) Dhi(1))
  (NO checksum in observed stream)
- Robust extractor: prefer exact-length using N; fallback header->header for sync/recovery.
- Accumulate sectors into one rotation using wrap-detection on start angle.
- Publish /scan_raw (assembled sectors for rotation) and /scan (resampled + adaptive EMA).
- Broadcast TF base_link -> laser aeach tick for RViz/SensorMessageFilter.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster

import serial, time, math, bisect
from threading import Thread, Lock

# --------------------
# Utilities
# --------------------
def u16_le(buf, off):
    return buf[off] | (buf[off+1] << 8)

def clamp_float(x, mn, mx):
    return max(mn, min(mx, x))

# --------------------
# Node
# --------------------
class LDS50CR(Node):
    def __init__(self):
        super().__init__('lds50cr')

        # --- parameters (tweakable) ---
        self.declare_parameter('port', '/dev/cp210x_lidar')
        self.declare_parameter('baud', 921600)
        self.declare_parameter('laser_frame', 'laser')
        self.declare_parameter('min_range', 0.3)
        self.declare_parameter('max_range', 40.0)

        # publish one rotation by default
        self.declare_parameter('publish_mode', 'rotation')  # 'rotation' or 'coverage'
        self.declare_parameter('accumulate_deg', 360.0)     # used when coverage mode
        self.declare_parameter('packet_timeout', 0.25)      # how long to wait to finish rotation

        # resample + smoothing
        self.declare_parameter('resample_angle_increment_deg', 0.5)  # 0 => no resample
        self.declare_parameter('smoothing_enabled', True)
        self.declare_parameter('smoothing_alpha_min', 0.12)
        self.declare_parameter('smoothing_alpha_max', 0.9)
        self.declare_parameter('smoothing_small_delta', 0.02)
        self.declare_parameter('max_jump_m', 0.8)

        # wrap detection
        self.declare_parameter('wrap_threshold_deg', 10.0)  # detect new rotation if start drops by > this

        # read params
        p = self.get_parameter
        self.port = p('port').value
        self.baud = int(p('baud').value)
        self.laser_frame = p('laser_frame').value
        self.min_range = float(p('min_range').value)
        self.max_range = float(p('max_range').value)

        self.publish_mode = p('publish_mode').value
        self.accumulate_deg = float(p('accumulate_deg').value)
        self.packet_timeout = float(p('packet_timeout').value)

        self.resample_inc = float(p('resample_angle_increment_deg').value)
        self.smoothing_enabled = bool(p('smoothing_enabled').value)
        self.alpha_min = float(p('smoothing_alpha_min').value)
        self.alpha_max = float(p('smoothing_alpha_max').value)
        self.small_delta = float(p('smoothing_small_delta').value)
        self.max_jump = float(p('max_jump_m').value)

        self.wrap_threshold = float(p('wrap_threshold_deg').value)

        self.get_logger().info(f"LDS50CR init {self.port}@{self.baud} frame={self.laser_frame}")

        # publishers
        self.pub_raw = self.create_publisher(LaserScan, 'scan_raw', 10)
        self.pub = self.create_publisher(LaserScan, 'scan', 10)

        # TF broadcaster
        # self.tf_broadcaster = TransformBroadcaster(self)

        # serial open
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.2)
        except Exception as e:
            self.get_logger().error(f"Cannot open serial {self.port}: {e}")
            raise

        # buffer/thread
        self.buf = bytearray()
        self.lock = Lock()
        self.running = True
        Thread(target=self._reader_thread, daemon=True).start()

        # state for assembly
        self._collected_pkts = []    # list of parsed packet dicts for current rotation
        self._last_start = None      # last seen packet start angle (deg)
        self._rotation_start_time = None

        # smoothing state
        self.prev_resampled = None

        # timer
        self.timer = self.create_timer(0.03, self._timer_cb)  # ~33 Hz tick

    # --------------------
    # serial reader thread
    # --------------------
    def _reader_thread(self):
        while self.running and rclpy.ok():
            try:
                data = self.ser.read(8192)
                if data:
                    with self.lock:
                        self.buf.extend(data)
                else:
                    time.sleep(0.002)
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

    # --------------------
    # parse packet bytes -> dict (expects full packet bytes)
    # Format observed: CF FA | N(2) | start(2) | sector(2) | N*(I,Dlo,Dhi)
    # Return None if invalid/incomplete
    # --------------------
    def _parse_packet_from_raw(self, raw):
        if len(raw) < 8:
            return None
        if raw[0] != 0xCF or raw[1] != 0xFA:
            return None
        if len(raw) < 8:
            return None
        N = u16_le(raw, 2)
        start_raw = u16_le(raw, 4)
        sector_raw = u16_le(raw, 6)
        need = 8 + N * 3  # no checksum in observed stream
        if len(raw) < need:
            return None
        pts = []
        off = 8
        for _ in range(N):
            inten = raw[off]
            dist = raw[off+1] | (raw[off+2] << 8)
            pts.append((int(inten), int(dist)))
            off += 3
        return {
            'N': N,
            'start_raw': start_raw,
            'sector_raw': sector_raw,
            'points': pts,
            'raw_len': need
        }

    # --------------------
    # robust extractor hybrid: prefer exact length by N; fallback header->header to recover
    # returns list of parsed packet dicts
    # --------------------
    def _extract_packets(self):
        hdr = b'\xCF\xFA'
        with self.lock:
            data = bytes(self.buf)
        L = len(data)
        pos = 0
        packets = []
        consumed = 0

        while pos < L - 1:
            h1 = data.find(hdr, pos)
            if h1 < 0:
                break
            # ensure we can read header fields
            if h1 + 8 <= L:
                N = u16_le(data, h1 + 2)
                need = 8 + N * 3
                if h1 + need <= L:
                    raw_pkt = data[h1:h1+need]
                    pkt = self._parse_packet_from_raw(raw_pkt)
                    if pkt:
                        packets.append(pkt)
                    pos = h1 + need
                    consumed = pos
                    continue
                else:
                    # can't read full-length yet; look for next header
                    h2 = data.find(hdr, h1 + 2)
                    if h2 >= 0:
                        raw_pkt = data[h1:h2]
                        pkt = self._parse_packet_from_raw(raw_pkt)
                        if pkt:
                            packets.append(pkt)
                        pos = h2
                        consumed = pos
                        continue
                    else:
                        break
            else:
                # can't read N yet; maybe header split; try find next header
                h2 = data.find(hdr, h1 + 2)
                if h2 >= 0:
                    raw_pkt = data[h1:h2]
                    pkt = self._parse_packet_from_raw(raw_pkt)
                    if pkt:
                        packets.append(pkt)
                    pos = h2
                    consumed = pos
                    continue
                else:
                    break

        # consume used bytes
        if consumed > 0:
            with self.lock:
                del self.buf[:consumed]
        return packets

    # --------------------
    # assemble list of packets -> sorted angles (deg), ranges (m), intensities
    # sector_raw observed is delta angle (0.1deg)
    # --------------------
    def _assemble_pkts_to_points(self, pkts):
        if not pkts:
            return None
        pts = []
        t0 = None
        t1 = None
        for pkt in pkts:
            N = pkt['N']
            start_deg = pkt['start_raw'] * 0.1
            sector_deg = pkt['sector_raw'] * 0.1  # delta angle
            # handle deg normalization
            if sector_deg < 0:
                sector_deg += 360.0
            step = (sector_deg / (N - 1)) if N > 1 else 0.0
            for i, (inten, dist_mm) in enumerate(pkt['points']):
                angle = (start_deg + i * step) % 360.0
                r = dist_mm / 1000.0
                pts.append((angle, r, float(inten)))
                now = time.time()
                if t0 is None:
                    t0 = now
                t1 = now
        if not pts:
            return None
        # sort by angle ascending to build a nice LaserScan
        pts.sort(key=lambda x: x[0])
        angles = [p[0] for p in pts]
        ranges = [p[1] for p in pts]
        intens = [p[2] for p in pts]
        coverage = angles[-1] - angles[0]
        if coverage < 0:
            coverage += 360.0
        scan_time = max(0.02, (t1 - t0) if (t0 and t1 and (t1 - t0) > 0) else 0.02)
        return {
            'angles': angles,
            'ranges': ranges,
            'intens': intens,
            'coverage': coverage,
            'scan_time': scan_time
        }

    # --------------------
    # resample uniformly (deg) across 0..360
    # --------------------
    def _resample_uniform(self, angles_deg, ranges_m, intens, ang_inc_deg):
        if ang_inc_deg <= 0 or len(angles_deg) < 2:
            return angles_deg, ranges_m, intens
        step = ang_inc_deg
        n = max(2, int(math.ceil(360.0 / step)))
        target = [i * step for i in range(n)]
        A = angles_deg + [a + 360.0 for a in angles_deg]
        R = ranges_m + ranges_m[:]
        I = intens + intens[:]
        out_r = []
        out_i = []
        for t in target:
            idx = bisect.bisect_left(A, t)
            if idx == 0:
                out_r.append(R[0]); out_i.append(I[0])
            elif idx >= len(A):
                out_r.append(R[-1]); out_i.append(I[-1])
            else:
                a0, a1 = A[idx-1], A[idx]
                r0, r1 = R[idx-1], R[idx]
                i0, i1 = I[idx-1], I[idx]
                frac = 0.0 if a1 == a0 else (t - a0) / (a1 - a0)
                rr = (1 - frac) * r0 + frac * r1
                ii = (1 - frac) * i0 + frac * i1
                out_r.append(rr); out_i.append(ii)
        return target, out_r, out_i

    # --------------------
    # adaptive EMA smoothing (per beam)
    # - preserve immediate 'inf' when no data to avoid ghosts
    # --------------------
    def _adaptive_smooth(self, arr):
        # normalize to floats and inf
        norm = []
        for v in arr:
            if v is None or (isinstance(v, float) and math.isnan(v)) or v <= 0.0:
                norm.append(float('inf'))
            else:
                norm.append(float(v))
        if self.prev_resampled is None or len(self.prev_resampled) != len(norm):
            self.prev_resampled = norm[:]
            return norm[:]
        out = []
        for prev, cur in zip(self.prev_resampled, norm):
            if cur == float('inf'):
                out.append(float('inf'))
                continue
            if prev == float('inf'):
                out.append(cur)
                continue
            delta = abs(cur - prev)
            rel = delta / max(prev, 0.05)
            alpha = self.alpha_min + (self.alpha_max - self.alpha_min) * min(1.0, rel)
            if delta < self.small_delta:
                alpha = self.alpha_min * 0.6
            filtered = alpha * cur + (1.0 - alpha) * prev
            # clamp jump
            if abs(filtered - prev) > self.max_jump:
                if filtered > prev:
                    filtered = prev + self.max_jump
                else:
                    filtered = prev - self.max_jump
            out.append(filtered)
        self.prev_resampled = out[:]
        return out

    # --------------------
    # build LaserScan message from arrays (angles in degrees)
    # --------------------
    def _build_laserscan_msg(self, angles_deg, ranges_m, intens, scan_time):
        if not angles_deg or not ranges_m:
            return None
        a0 = angles_deg[0]
        a1 = angles_deg[-1]
        if a1 < a0:
            a1 += 360.0
        n = len(ranges_m)
        angle_min = math.radians(a0)
        angle_max = math.radians(a1)
        angle_increment = 0.0 if n <= 1 else (angle_max - angle_min) / (n - 1)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.laser_frame
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = scan_time / max(1, n - 1)
        msg.scan_time = scan_time
        msg.range_min = self.min_range
        msg.range_max = self.max_range
        # clamp ranges
        rr = []
        for r in ranges_m:
            if r is None or math.isnan(r) or r <= 0.0 or r < self.min_range or r > self.max_range:
                rr.append(float('inf'))
            else:
                rr.append(float(r))
        msg.ranges = rr
        msg.intensities = [float(x) for x in intens] if intens else []
        return msg

    # # --------------------
    # # broadcast TF base_link -> laser
    # # --------------------
    # def _send_tf(self):
    #     t = TransformStamped()
    #     t.header.stamp = self.get_clock().now().to_msg()
    #     t.header.frame_id = 'base_link'
    #     t.child_frame_id = self.laser_frame
    #     t.transform.translation.x = 0.0
    #     t.transform.translation.y = 0.0
    #     t.transform.translation.z = 0.0
    #     t.transform.rotation.x = 0.0
    #     t.transform.rotation.y = 0.0
    #     t.transform.rotation.z = 0.0
    #     t.transform.rotation.w = 1.0
    #     self.tf_broadcaster.sendTransform(t)

    # --------------------
    # main timer: extract packets, accumulate sectors into rotation using wrap detection
    # --------------------
    def _timer_cb(self):
        # broadcast TF first so rviz/msg filters find it
        # self._send_tf()

        # extract newly arrived packets
        new_pkts = self._extract_packets()
        if not new_pkts:
            # if timeout waiting for rotation completion, maybe publish partial (optional)
            # if we have partial and timeout exceeded, force publish to avoid stalling
            if self._rotation_start_time and (time.time() - self._rotation_start_time) > self.packet_timeout:
                if self._collected_pkts:
                    self.get_logger().warning("Rotation timeout — publishing partial rotation")
                    self._publish_collected_rotation()
            return

        # iterate over new packets in arrival order, detect wrap boundaries
        for pkt in new_pkts:
            start_deg = (pkt['start_raw'] * 0.1) % 360.0
            # first packet overall
            if not self._collected_pkts:
                self._collected_pkts.append(pkt)
                self._last_start = start_deg
                self._rotation_start_time = time.time()
                continue

            # compute delta from last_start in circular sense
            last = self._last_start
            # normalized delta in (-180, 180]
            delta = ((start_deg - last + 540.0) % 360.0) - 180.0

            # if start dropped by more than wrap_threshold → new rotation detected
            if (start_deg < last - self.wrap_threshold) or (delta < -self.wrap_threshold):
                # rotation boundary -> publish previous rotation and start new one
                self.get_logger().debug(f"Rotation boundary detected: last={last:.2f} start={start_deg:.2f} delta={delta:.2f}")
                self._publish_collected_rotation()
                # start new rotation with current pkt
                self._collected_pkts = [pkt]
                self._last_start = start_deg
                self._rotation_start_time = time.time()
            else:
                # otherwise append to current rotation
                self._collected_pkts.append(pkt)
                self._last_start = start_deg

            # optional early publish if using coverage mode and coverage reached
            if self.publish_mode != 'rotation':
                assembled = self._assemble_pkts_to_points(self._collected_pkts)
                if assembled and assembled['coverage'] >= self.accumulate_deg:
                    self.get_logger().debug("Coverage threshold reached — publishing")
                    self._publish_collected_rotation()
                    # reset for next rotation
                    self._collected_pkts = []
                    self._last_start = None
                    self._rotation_start_time = None

    # --------------------
    # publish helper: assemble current collected pkts and publish raw & resampled
    # --------------------
    def _publish_collected_rotation(self):
        if not self._collected_pkts:
            return
        assembled = self._assemble_pkts_to_points(self._collected_pkts)
        if assembled is None:
            return

        # publish raw
        raw_msg = self._build_laserscan_msg(assembled['angles'], assembled['ranges'], assembled['intens'], assembled['scan_time'])
        if raw_msg:
            self.pub_raw.publish(raw_msg)

        # resample
        if self.resample_inc > 0:
            tgt_ang, tgt_rng, tgt_int = self._resample_uniform(assembled['angles'], assembled['ranges'], assembled['intens'], self.resample_inc)
        else:
            tgt_ang, tgt_rng, tgt_int = assembled['angles'], assembled['ranges'], assembled['intens']

        # smoothing
        if self.smoothing_enabled:
            tgt_rng_sm = self._adaptive_smooth(tgt_rng)
        else:
            tgt_rng_sm = tgt_rng

        scan_msg = self._build_laserscan_msg(tgt_ang, tgt_rng_sm, tgt_int, assembled['scan_time'])
        if scan_msg:
            self.pub.publish(scan_msg)

        #self.get_logger().info(f"Published rotation: pts={len(assembled['angles'])} cov={assembled['coverage']:.1f}° packets={len(self._collected_pkts)}")
        # keep prev_resampled as-is to allow temporal smoothing continuity across rotations
        # If you prefer clearing smoothing per-rotation, uncomment:
        # self.prev_resampled = None

    # --------------------
    def destroy_node(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()

# --------------------
def main(args=None):
    rclpy.init(args=args)
    node = LDS50CR()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
