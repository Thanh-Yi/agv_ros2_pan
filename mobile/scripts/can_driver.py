import struct

class CANDriver:
    def __init__(self, ser, config):
        self.ser = ser
        self.cfg = config

    def _write_i32(self, node, index, sub, val):
        b = struct.pack("<i", int(val))
        payload = bytes([0x23, index & 0xFF, index >> 8, sub,
                         b[0], b[1], b[2], b[3]])
        self._send(node, payload)

    def _write_u16(self, node, index, sub, val):
        payload = bytes([0x2B, index & 0xFF, index >> 8, sub,
                         val & 0xFF, (val >> 8) & 0xFF, 0, 0])
        self._send(node, payload)

    def _write_u8(self, node, index, sub, val):
        payload = bytes([0x2F, index & 0xFF, index >> 8, sub,
                         val & 0xFF, 0, 0, 0])
        self._send(node, payload)

    def _send(self, node, data):
        can_id = 0x600 + node
        frame = bytes([0xAA, 0xC8, can_id & 0xFF, can_id >> 8]) + data + bytes([0x55])
        self.ser.write(frame)

    # ===== API =====
    def init_speed_mode(self, node):
        idx_ctrl, sub_ctrl = self.cfg.reg("controlword")
        idx_mode, sub_mode = self.cfg.reg("mode")

        self._write_u16(node, idx_ctrl, sub_ctrl, 0x0086)
        self._write_u8(node, idx_mode, sub_mode, 3)
        self._write_u16(node, idx_ctrl, sub_ctrl, 0x000F)

    def set_velocity(self, node, dec):
        idx, sub = self.cfg.reg("target_velocity")
        self._write_i32(node, idx, sub, dec)

    def set_deceleration(self, node, val):
        idx, sub = self.cfg.reg("deceleration")
        self._write_i32(node, idx, sub, val)