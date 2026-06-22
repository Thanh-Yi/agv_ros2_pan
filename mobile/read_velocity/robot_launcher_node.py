#!/usr/bin/env python3
"""
robot_launcher_node.py — chạy thường trực trong tmux "webui".

Services:
  /robot/start_slam  (std_srvs/srv/Trigger) → chạy bring_slam_v2.sh
  /robot/start_nav   (std_srvs/srv/Trigger) → chạy bring_it_up_all.sh
  /robot/stop        (std_srvs/srv/Trigger) → kill tmux "mobile"

Topics publish:
  /robot/mode            (std_msgs/String)       — "idle|launching_slam|launching_nav|slam|navigation"
  /map_editor/maps_list  (std_msgs/String)       — JSON list tên map trong MAP_DIR
  /map_editor/map        (nav_msgs/OccupancyGrid)— map data khi web yêu cầu load

Topics subscribe:
  /map_editor/select     (std_msgs/String)       — tên map cần load cho editor
"""

import os
import json
import threading
import subprocess

import yaml as yaml_lib
import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

# ── Cấu hình đường dẫn ───────────────────────────────────────────
WS          = "/home/pan/ros2_ws"
SLAM_SCRIPT = f"{WS}/src/mobile/read_velocity/bring_slam_v2.sh"
# NAV_SCRIPT  = f"{WS}/src/mobile/read_velocity/bring_it_up_all.sh"
NAV_SCRIPT  = f"{WS}/src/mobile/read_velocity/bring_it_up_not_camera.sh"
SESSION     = "mobile"
MAP_DIR     = "/home/pan/ros2_ws/src/mobile/map"


class RobotLauncherNode(Node):

    def __init__(self):
        super().__init__("robot_launcher")
        self._lock        = threading.Lock()
        self._launching   = False
        self._launch_mode = ""   # "slam" | "nav"

        # ── Services ─────────────────────────────────────────────
        self.create_service(Trigger, "/robot/start_slam", self._start_slam)
        self.create_service(Trigger, "/robot/start_nav",  self._start_nav)
        self.create_service(Trigger, "/robot/stop",       self._stop)

        # ── Robot mode publisher ──────────────────────────────────
        self._mode_pub = self.create_publisher(String, "/robot/mode", 10)
        self.create_timer(2.0, self._publish_mode)

        # ── Map editor publishers / subscriber ────────────────────
        self._map_editor_pub = self.create_publisher(
            OccupancyGrid, "/map_editor/map", 1)
        self._maps_list_pub = self.create_publisher(
            String, "/map_editor/maps_list", 10)
        self._map_select_sub = self.create_subscription(
            String, "/map_editor/select", self._on_map_select, 10)
        self.create_timer(5.0, self._publish_maps_list)

        self.get_logger().info("robot_launcher_node sẵn sàng")
        self.get_logger().info(f"  SLAM : {SLAM_SCRIPT}")
        self.get_logger().info(f"  NAV  : {NAV_SCRIPT}")
        self.get_logger().info(f"  MAP  : {MAP_DIR}")

    # ════════════════════════════════════════════════════════════
    # TMUX HELPERS
    # ════════════════════════════════════════════════════════════

    def _tmux_has_session(self):
        r = subprocess.run(
            ["tmux", "has-session", "-t", SESSION],
            capture_output=True)
        return r.returncode == 0

    def _tmux_has_window(self, window):
        r = subprocess.run(
            ["tmux", "has-session", "-t", f"{SESSION}:{window}"],
            capture_output=True)
        return r.returncode == 0

    def _detect_mode(self):
        with self._lock:
            launching   = self._launching
            launch_mode = self._launch_mode
        if launching:
            return f"launching_{launch_mode}"
        if not self._tmux_has_session():
            return "idle"
        if self._tmux_has_window("slam"):
            return "slam"
        if self._tmux_has_window("bringup"):
            return "navigation"
        return "unknown"

    # ════════════════════════════════════════════════════════════
    # ROBOT MODE SERVICES
    # ════════════════════════════════════════════════════════════

    def _start_slam(self, req, res):
        mode = self._detect_mode()
        if mode == "slam":
            res.success, res.message = True,  "SLAM đang chạy rồi"
            return res
        if mode in ("navigation", "launching_nav"):
            res.success, res.message = False, "Navigation đang chạy — dừng trước khi chạy SLAM"
            return res
        if mode == "launching_slam":
            res.success, res.message = False, "Đang khởi động SLAM, vui lòng chờ..."
            return res
        with self._lock:
            self._launching   = True
            self._launch_mode = "slam"
        threading.Thread(target=self._run_script, args=(SLAM_SCRIPT,), daemon=True).start()
        res.success, res.message = True, "Đang khởi động SLAM (~2-10 phút)"
        self.get_logger().info("Bắt đầu khởi động SLAM")
        return res

    def _start_nav(self, req, res):
        mode = self._detect_mode()
        if mode == "navigation":
            res.success, res.message = True,  "Navigation đang chạy rồi"
            return res
        if mode in ("slam", "launching_slam"):
            res.success, res.message = False, "SLAM đang chạy — dừng trước khi chạy Navigation"
            return res
        if mode == "launching_nav":
            res.success, res.message = False, "Đang khởi động Navigation, vui lòng chờ..."
            return res
        with self._lock:
            self._launching   = True
            self._launch_mode = "nav"
        threading.Thread(target=self._run_script, args=(NAV_SCRIPT,), daemon=True).start()
        res.success, res.message = True, "Đang khởi động Navigation (~4-15 phút)"
        self.get_logger().info("Bắt đầu khởi động Navigation")
        return res

    def _stop(self, req, res):
        if self._tmux_has_session():
            subprocess.run(["tmux", "kill-session", "-t", SESSION])
            with self._lock:
                self._launching   = False
                self._launch_mode = ""
            res.success, res.message = True,  f"Đã dừng — tmux '{SESSION}' đã kill"
            self.get_logger().info("Đã kill tmux mobile")
        else:
            res.success, res.message = False, "Không có gì đang chạy"
        return res

    def _run_script(self, script):
        try:
            result = subprocess.run(["bash", script], timeout=900)
            if result.returncode == 0:
                self.get_logger().info(f"Script hoàn thành: {script}")
            else:
                self.get_logger().error(f"Script lỗi (code {result.returncode})")
        except subprocess.TimeoutExpired:
            self.get_logger().error("Script timeout sau 15 phút")
        except Exception as e:
            self.get_logger().error(f"Lỗi: {e}")
        finally:
            with self._lock:
                self._launching   = False
                self._launch_mode = ""

    def _publish_mode(self):
        self._mode_pub.publish(String(data=self._detect_mode()))

    # ════════════════════════════════════════════════════════════
    # MAP EDITOR
    # ════════════════════════════════════════════════════════════

    def _get_available_maps(self):
        try:
            files = os.listdir(MAP_DIR)
            return sorted(f[:-5] for f in files if f.endswith('.yaml'))
        except Exception:
            return []

    def _publish_maps_list(self):
        maps = self._get_available_maps()
        self._maps_list_pub.publish(String(data=json.dumps(maps)))

    def _on_map_select(self, msg):
        map_name = msg.data.strip()
        if not map_name:
            return
        self.get_logger().info(f"Web yêu cầu load map: '{map_name}'")
        threading.Thread(
            target=self._load_and_publish_map,
            args=(map_name,),
            daemon=True
        ).start()

    def _load_and_publish_map(self, map_name):
        try:
            yaml_path = os.path.join(MAP_DIR, f"{map_name}.yaml")
            if not os.path.exists(yaml_path):
                self.get_logger().error(f"Không tìm thấy: {yaml_path}")
                return

            with open(yaml_path, 'r') as f:
                meta = yaml_lib.safe_load(f)

            pgm_file = meta.get('image', f"{map_name}.pgm")
            pgm_path = pgm_file if os.path.isabs(pgm_file) \
                       else os.path.join(MAP_DIR, pgm_file)

            if not os.path.exists(pgm_path):
                self.get_logger().error(f"Không tìm thấy PGM: {pgm_path}")
                return

            # Đọc ảnh và flip (ROS origin ở bottom-left)
            img   = Image.open(pgm_path).convert('L')
            arr   = np.flipud(np.array(img))
            h, w  = arr.shape

            resolution  = float(meta.get('resolution', 0.05))
            origin      = meta.get('origin', [0.0, 0.0, 0.0])
            negate      = int(meta.get('negate', 0))
            occ_thresh  = float(meta.get('occupied_thresh', 0.65))
            free_thresh = float(meta.get('free_thresh', 0.196))

            # Chuyển pixel → occupancy value
            data = []
            for px in arr.flatten():
                val = int(px)
                if negate:
                    val = 255 - val
                occ = 1.0 - val / 254.0
                if occ > occ_thresh:
                    data.append(100)    # occupied
                elif occ < free_thresh:
                    data.append(0)      # free
                else:
                    data.append(-1)     # unknown

            # Build OccupancyGrid message
            grid = OccupancyGrid()
            grid.header.stamp            = self.get_clock().now().to_msg()
            grid.header.frame_id         = 'map'
            grid.info.resolution         = resolution
            grid.info.width              = w
            grid.info.height             = h
            grid.info.origin.position.x  = float(origin[0])
            grid.info.origin.position.y  = float(origin[1])
            grid.info.origin.position.z  = 0.0
            grid.data                    = data

            self._map_editor_pub.publish(grid)
            self.get_logger().info(
                f"Đã publish map '{map_name}' ({w}x{h}) → /map_editor/map")

        except Exception as e:
            self.get_logger().error(f"Lỗi load map '{map_name}': {e}")


# ════════════════════════════════════════════════════════════════
# ENTRY POINT
# ════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = RobotLauncherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
