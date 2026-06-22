#!/usr/bin/env python3
"""
camera_manager_node.py
Services:
  /robot/set_camera_high  (std_srvs/srv/SetBool)  — RealSense D4xx (camera_high)
  /robot/set_camera_low   (std_srvs/srv/SetBool)  — Orbbec          (camera_low)
"""

import shlex
import subprocess
import threading

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class CameraManager(Node):
    def __init__(self):
        super().__init__('camera_manager')

        # ── camera_high: RealSense ───────────────────────────────────
        self.declare_parameter(
            'camera_high_cmd',
            'ros2 launch realsense2_camera rs_launch.py'
            ' camera_namespace:=camera_high'
            ' camera_name:=camera_high'
            ' enable_color:=true'
            ' enable_depth:=false'
            ' align_depth.enable:=false',
        )
        # Republish raw → compressed để web subscribe được
        self.declare_parameter(
            'camera_high_republish_cmd',
            'ros2 run image_transport republish raw compressed'
            ' --ros-args'
            ' --remap in:=/camera_high/camera_high/color/image_raw'
            ' --remap out/compressed:=/camera_high/camera_high/color/image_raw/compressed',
        )

        # ── camera_low: Orbbec ───────────────────────────────────────
        # Orbbec publish compressed trực tiếp nên không cần republish
        self.declare_parameter(
            'camera_low_cmd',
            'ros2 launch orbbec_camera gemini_e.launch.py'
            ' camera_name:=camera_low'
            ' enable_color:=true'
            ' enable_depth:=true'
            ' enable_ir:=true',
        )

        # _procs[cam] = list các Popen đang chạy cho camera đó
        self._procs: dict[str, list[subprocess.Popen]] = {
            'high': [],
            'low':  [],
        }

        self.create_service(
            SetBool, '/robot/set_camera_high',
            lambda req, resp: self._handle('high', req, resp),
        )
        self.create_service(
            SetBool, '/robot/set_camera_low',
            lambda req, resp: self._handle('low', req, resp),
        )

        self.get_logger().info('CameraManager ready')

    def _cmds_for(self, cam: str) -> list[str]:
        """Danh sách command cần spawn cho camera (theo thứ tự)."""
        if cam == 'high':
            return [
                self.get_parameter('camera_high_cmd').value,
                self.get_parameter('camera_high_republish_cmd').value,
            ]
        return [
            self.get_parameter('camera_low_cmd').value,
        ]

    def _is_running(self, cam: str) -> bool:
        return bool(self._procs[cam]) and any(
            p.poll() is None for p in self._procs[cam]
        )

    def _handle(self, cam: str, req: SetBool.Request, resp: SetBool.Response):
        if req.data:  # ── BẬT ──
            if not self._is_running(cam):
                procs = []
                for cmd in self._cmds_for(cam):
                    self.get_logger().info(f'[{cam}] starting: {cmd}')
                    procs.append(subprocess.Popen(shlex.split(cmd)))
                self._procs[cam] = procs
                resp.success = True
                resp.message = f'camera_{cam} started ({len(procs)} processes)'
            else:
                resp.success = True
                resp.message = 'already running'

        else:  # ── TẮT — terminate trong thread, không block executor ──
            procs, self._procs[cam] = self._procs[cam], []
            if procs:
                threading.Thread(
                    target=self._stop_procs,
                    args=(cam, procs),
                    daemon=True,
                ).start()
            resp.success = True
            resp.message = f'camera_{cam} stopped'

        return resp

    def _stop_procs(self, cam: str, procs: list[subprocess.Popen]):
        """Gửi SIGTERM đồng thời tất cả, rồi chờ từng process."""
        for p in procs:
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                p.kill()
                try:
                    p.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
        self.get_logger().info(f'[{cam}] all processes terminated')

    def destroy_node(self):
        """Tắt tất cả camera khi node bị kill."""
        for cam, procs in self._procs.items():
            if procs:
                threading.Thread(
                    target=self._stop_procs,
                    args=(cam, procs),
                    daemon=True,
                ).start()
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
