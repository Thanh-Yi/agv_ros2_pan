#!/usr/bin/env python3
"""
ROS 2 backend for Web Dashboard Mapping page.

It connects the web buttons to ROS 2:
- Subscribes: /web_mapping_command (std_msgs/String)
- Publishes : /web_mapping_status  (std_msgs/String)
- Republishes SLAM map to: /web_map (nav_msgs/OccupancyGrid)

Commands from web:
- START_MAPPING
- STOP_MAPPING
- SAVE_MAP:<map_name>

Default behavior:
- Starts slam_toolbox async_slam_toolbox_node.
- Remaps slam_toolbox map output from /map to /slam_map so it does not mix with Nav2 map_server /map.
- Republishes /slam_map to /web_map for the web Mapping page.
- Saves the latest received OccupancyGrid directly as .pgm + .yaml into
  /home/pan/ros2_ws/src/mobile/map by default.

Why direct saving:
- On some Jetson / Nav2 setups, nav2_map_server map_saver_cli can fail with
  "Failed to spin map" because of QoS/topic timing.
- This backend already receives the live SLAM map, so it writes the PGM/YAML
  itself without waiting for map_saver_cli.
"""

import math
import os
import shlex
import signal
import subprocess
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


class WebMappingBackend(Node):
    def __init__(self) -> None:
        super().__init__("web_mapping_backend")

        self.declare_parameter("slam_scan_topic", "/scan_filtered")
        self.declare_parameter("fallback_scan_topic", "/scan")
        self.declare_parameter("slam_map_topic", "/slam_map")
        self.declare_parameter("web_map_topic", "/web_map")
        self.declare_parameter("map_save_dir", "/home/pan/ros2_ws/src/mobile/map")
        self.declare_parameter("use_sim_time_for_slam", False)
        self.declare_parameter("slam_params_file", "")
        self.declare_parameter("start_slam_cmd", "")
        self.declare_parameter("stop_after_save", True)

        self.slam_scan_topic = str(self.get_parameter("slam_scan_topic").value)
        self.fallback_scan_topic = str(self.get_parameter("fallback_scan_topic").value)
        self.slam_map_topic = str(self.get_parameter("slam_map_topic").value)
        self.web_map_topic = str(self.get_parameter("web_map_topic").value)
        self.map_save_dir = os.path.expanduser(str(self.get_parameter("map_save_dir").value))
        self.use_sim_time_for_slam = bool(self.get_parameter("use_sim_time_for_slam").value)
        self.slam_params_file = str(self.get_parameter("slam_params_file").value)
        self.start_slam_cmd = str(self.get_parameter("start_slam_cmd").value)
        self.stop_after_save = bool(self.get_parameter("stop_after_save").value)

        self.slam_proc: Optional[subprocess.Popen] = None
        self.last_map_time: Optional[float] = None
        self.last_map_msg: Optional[OccupancyGrid] = None

        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.status_pub = self.create_publisher(String, "/web_mapping_status", status_qos)
        self.web_map_pub = self.create_publisher(OccupancyGrid, self.web_map_topic, map_qos)

        self.create_subscription(String, "/web_mapping_command", self.on_command, 10)
        self.create_subscription(OccupancyGrid, self.slam_map_topic, self.on_slam_map, map_qos)

        self.create_timer(2.0, self.watchdog)

        os.makedirs(self.map_save_dir, exist_ok=True)
        self.publish_status("WEB_MAPPING_BACKEND_READY")
        self.get_logger().info("Web mapping backend ready.")
        self.get_logger().info("Listening command: /web_mapping_command")
        self.get_logger().info(f"SLAM map topic: {self.slam_map_topic} -> Web map topic: {self.web_map_topic}")
        self.get_logger().info(f"Map save directory: {self.map_save_dir}")

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def topic_exists(self, topic_name: str) -> bool:
        try:
            topics = dict(self.get_topic_names_and_types())
            return topic_name in topics
        except Exception:
            return False

    def choose_scan_topic(self) -> str:
        if self.topic_exists(self.slam_scan_topic):
            return self.slam_scan_topic
        if self.topic_exists(self.fallback_scan_topic):
            return self.fallback_scan_topic
        return self.slam_scan_topic

    def build_slam_cmd(self) -> List[str]:
        if self.start_slam_cmd.strip():
            return shlex.split(self.start_slam_cmd)

        scan_topic = self.choose_scan_topic()
        cmd = [
            "ros2", "run", "slam_toolbox", "async_slam_toolbox_node",
            "--ros-args",
        ]

        if self.slam_params_file.strip():
            params_file = os.path.expanduser(self.slam_params_file.strip())
            if os.path.exists(params_file):
                cmd += ["--params-file", params_file]
            else:
                self.publish_status(f"SLAM_PARAMS_FILE_NOT_FOUND:{params_file}")

        cmd += [
            "-p", f"use_sim_time:={str(self.use_sim_time_for_slam).lower()}",
            "-r", f"scan:={scan_topic}",
            "-r", f"map:={self.slam_map_topic}",
        ]
        return cmd

    def start_mapping(self) -> None:
        if self.slam_proc is not None and self.slam_proc.poll() is None:
            self.publish_status("START_MAPPING_ALREADY_RUNNING")
            return

        self.last_map_time = None
        self.last_map_msg = None
        cmd = self.build_slam_cmd()
        self.publish_status("START_MAPPING_REQUESTED")
        self.get_logger().info("Starting SLAM command: " + " ".join(shlex.quote(x) for x in cmd))

        try:
            self.slam_proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            self.publish_status(f"START_MAPPING_STARTED:{self.slam_map_topic}_TO_{self.web_map_topic}")
        except FileNotFoundError:
            self.slam_proc = None
            self.publish_status("START_MAPPING_FAILED:ros2_COMMAND_NOT_FOUND")
        except Exception as exc:
            self.slam_proc = None
            self.publish_status(f"START_MAPPING_FAILED:{type(exc).__name__}:{exc}")

    def stop_mapping(self) -> None:
        if self.slam_proc is None or self.slam_proc.poll() is not None:
            self.slam_proc = None
            self.publish_status("STOP_MAPPING_ALREADY_STOPPED")
            return

        self.publish_status("STOP_MAPPING_REQUESTED")
        try:
            os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGINT)
            try:
                self.slam_proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGTERM)
                try:
                    self.slam_proc.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(self.slam_proc.pid), signal.SIGKILL)
            self.publish_status("STOP_MAPPING_STOPPED")
        except Exception as exc:
            self.publish_status(f"STOP_MAPPING_FAILED:{type(exc).__name__}:{exc}")
        finally:
            self.slam_proc = None

    @staticmethod
    def quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def occupancy_to_pgm_value(occ: int) -> int:
        # Same practical convention as Nav2 map_saver in trinary mode:
        # occupied = black, free = white, unknown = gray.
        if occ < 0:
            return 205
        if occ >= 65:
            return 0
        if occ <= 25:
            return 254
        return 205

    def write_map_direct(self, grid: OccupancyGrid, output_path_no_ext: str) -> None:
        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        data = list(grid.data)

        if width <= 0 or height <= 0:
            raise RuntimeError(f"invalid map size {width}x{height}")
        if len(data) != width * height:
            raise RuntimeError(f"invalid map data length {len(data)} for {width}x{height}")

        pgm_path = output_path_no_ext + ".pgm"
        yaml_path = output_path_no_ext + ".yaml"
        image_name = os.path.basename(pgm_path)

        # OccupancyGrid origin is bottom-left in map coordinates, while PGM rows
        # are written top-to-bottom, so rows must be vertically flipped.
        with open(pgm_path, "wb") as f:
            f.write(f"P5\n# CREATOR: web_mapping_backend.py {resolution:.6f} m/pix\n{width} {height}\n255\n".encode("ascii"))
            for y in range(height - 1, -1, -1):
                row_start = y * width
                row = bytearray(self.occupancy_to_pgm_value(int(v)) for v in data[row_start:row_start + width])
                f.write(row)

        origin = grid.info.origin
        yaw = self.quaternion_to_yaw(origin.orientation)
        with open(yaml_path, "w", encoding="utf-8") as f:
            f.write(f"image: {image_name}\n")
            f.write("mode: trinary\n")
            f.write(f"resolution: {resolution:.6f}\n")
            f.write(f"origin: [{origin.position.x:.6f}, {origin.position.y:.6f}, {yaw:.6f}]\n")
            f.write("negate: 0\n")
            f.write("occupied_thresh: 0.65\n")
            f.write("free_thresh: 0.25\n")

    def save_map(self, map_name: str) -> None:
        safe_name = "".join(c if c.isalnum() or c in ["_", "-"] else "_" for c in map_name).strip("_")
        if not safe_name:
            safe_name = time.strftime("web_map_%Y%m%d_%H%M%S")

        os.makedirs(self.map_save_dir, exist_ok=True)
        output_path = os.path.join(self.map_save_dir, safe_name)
        self.publish_status(f"SAVE_MAP_REQUESTED:{safe_name}")

        if self.last_map_msg is None:
            self.publish_status("SAVE_MAP_FAILED:NO_LIVE_SLAM_MAP_RECEIVED")
            return

        try:
            self.write_map_direct(self.last_map_msg, output_path)
            self.publish_status(f"SAVE_MAP_DONE:{output_path}.yaml")
            if self.stop_after_save:
                self.stop_mapping()
        except Exception as exc:
            self.publish_status(f"SAVE_MAP_FAILED:{type(exc).__name__}:{exc}")

    def on_command(self, msg: String) -> None:
        command = msg.data.strip()
        if command == "START_MAPPING":
            self.start_mapping()
        elif command == "STOP_MAPPING":
            self.stop_mapping()
        elif command.startswith("SAVE_MAP"):
            if ":" in command:
                _, map_name = command.split(":", 1)
            else:
                map_name = time.strftime("web_map_%Y%m%d_%H%M%S")
            self.save_map(map_name)
        else:
            self.publish_status(f"UNKNOWN_MAPPING_COMMAND:{command}")

    def on_slam_map(self, msg: OccupancyGrid) -> None:
        self.last_map_time = time.time()
        self.last_map_msg = msg
        self.web_map_pub.publish(msg)

    def watchdog(self) -> None:
        if self.slam_proc is not None and self.slam_proc.poll() is not None:
            code = self.slam_proc.returncode
            self.slam_proc = None
            self.publish_status(f"SLAM_PROCESS_EXITED:{code}")
            return

        if self.slam_proc is not None and self.slam_proc.poll() is None:
            if self.last_map_time is None:
                self.publish_status("MAPPING_WAITING_FOR_SLAM_MAP")


def main() -> None:
    rclpy.init()
    node = WebMappingBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_mapping()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
