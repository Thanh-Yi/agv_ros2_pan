#!/usr/bin/env python3
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def split_command(command: str) -> List[str]:
    return ["bash", "-lc", command]


def safe_name(name: str) -> str:
    name = name.strip()
    name = re.sub(r"\.ya?ml$", "", name)
    name = re.sub(r"[^A-Za-z0-9_\-]+", "_", name)
    name = name.strip("_")
    return name or time.strftime("web_map_%Y%m%d_%H%M%S")


class WebSystemManager(Node):
    def __init__(self):
        super().__init__("web_system_manager")

        self.map_dir = Path(os.environ.get(
            "WEB_MAP_DIR",
            "/home/pan/ros2_ws/src/mobile/map"
        )).expanduser()
        self.map_dir.mkdir(parents=True, exist_ok=True)

        self.current_map_name: Optional[str] = None
        self.current_map_path: Optional[Path] = None

        self.slam_process: Optional[subprocess.Popen] = None
        self.nav_process: Optional[subprocess.Popen] = None

        # Map preview mode: chỉ mở map_server để web thấy map,
        # chưa chạy AMCL/Nav2 điều hướng.
        self.map_server_process: Optional[subprocess.Popen] = None
        self.map_lifecycle_process: Optional[subprocess.Popen] = None

        self.system_status_pub = self.create_publisher(String, "/web_system_status", 10)
        self.mapping_status_pub = self.create_publisher(String, "/web_mapping_status", 10)
        self.map_list_pub = self.create_publisher(String, "/web_map_list", 10)

        self.create_subscription(String, "/web_system_command", self.on_system_command, 10)
        self.create_subscription(String, "/web_mapping_command", self.on_system_command, 10)

        self.publish_status("READY")
        self.publish_map_list()

        self.get_logger().info("Web System Manager is ready.")
        self.get_logger().info(f"Map directory: {self.map_dir}")

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.system_status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_mapping_status(self, text: str):
        msg = String()
        msg.data = text
        self.mapping_status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_map_list(self):
        maps = sorted([p.stem for p in self.map_dir.glob("*.yaml")])
        msg = String()
        msg.data = ",".join(maps)
        self.map_list_pub.publish(msg)
        self.get_logger().info(f"MAP_LIST:{msg.data}")

    def terminate_process(self, proc: Optional[subprocess.Popen], name: str, timeout: float = 5.0):
        if proc is None:
            return None

        if proc.poll() is not None:
            return None

        self.get_logger().info(f"Stopping {name} ...")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"{name} did not stop with SIGINT. Killing ...")
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=timeout)
            except Exception:
                pass
        except ProcessLookupError:
            pass
        except Exception as exc:
            self.get_logger().warn(f"Error stopping {name}: {exc}")

        return None

    def run_process(self, command: str, name: str) -> subprocess.Popen:
        self.get_logger().info(f"Starting {name}: {command}")
        return subprocess.Popen(
            split_command(command),
            preexec_fn=os.setsid
        )

    def run_quiet(self, command: str):
        subprocess.run(["bash", "-lc", command], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def is_running(self, proc: Optional[subprocess.Popen]) -> bool:
        return proc is not None and proc.poll() is None

    def is_navigation_running(self) -> bool:
        if self.is_running(self.nav_process):
            return True

        result = subprocess.run(
            ["bash", "-lc", "ros2 node list 2>/dev/null | grep -E '^/(amcl|bt_navigator|controller_server|planner_server|map_server|nav2_container)$'"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True
        )
        return bool(result.stdout.strip())

    def is_mapping_running(self) -> bool:
        if self.is_running(self.slam_process):
            return True

        result = subprocess.run(
            ["bash", "-lc", "ros2 node list 2>/dev/null | grep -E '^/slam_toolbox$'"],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True
        )
        return bool(result.stdout.strip())

    def stop_map_preview(self):
        self.map_lifecycle_process = self.terminate_process(self.map_lifecycle_process, "Map lifecycle manager")
        self.map_server_process = self.terminate_process(self.map_server_process, "Map server preview")

        self.run_quiet('pkill -f "ros2 run nav2_map_server map_server"')
        self.run_quiet('pkill -f "ros2 run nav2_lifecycle_manager lifecycle_manager"')
        self.publish_status("MAP_PREVIEW_STOPPED")

    def stop_navigation(self):
        # Dừng robot trước khi hạ Nav2 để tránh tiếp tục giữ lệnh vận tốc cũ.
        self.run_quiet("ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
        self.nav_process = self.terminate_process(self.nav_process, "Navigation")

        kill_cmds = [
            'pkill -f "ros2 launch mobile nav2.launch.py"',
            'pkill -f "map_server"',
            'pkill -f "amcl"',
            'pkill -f "nav2_container"',
            'pkill -f "planner_server"',
            'pkill -f "controller_server"',
            'pkill -f "bt_navigator"',
            'pkill -f "behavior_server"',
            'pkill -f "smoother_server"',
            'pkill -f "waypoint_follower"',
            'pkill -f "velocity_smoother"',
            'pkill -f "lifecycle_manager_localization"',
            'pkill -f "lifecycle_manager_navigation"',
        ]
        for cmd in kill_cmds:
            self.run_quiet(cmd)
        self.publish_status("NAVIGATION_STOPPED")

    def stop_mapping(self):
        self.slam_process = self.terminate_process(self.slam_process, "SLAM")
        self.run_quiet('pkill -f "slam_toolbox"')
        self.publish_status("MAPPING_STOPPED")
        self.publish_mapping_status("MAPPING_STOPPED")

    def start_mapping(self):
        # An toàn: không cho bật Mapping khi Navigation đang chạy.
        # Người dùng phải bấm Stop Navigation trước.
        if self.is_navigation_running():
            self.publish_status("START_MAPPING_BLOCKED:STOP_NAVIGATION_FIRST")
            self.publish_mapping_status("START_MAPPING_BLOCKED:STOP_NAVIGATION_FIRST")
            return

        # Mapping độc lập: tắt map preview trước để /map không bị map_server chiếm.
        self.stop_map_preview()

        slam_cmd = os.environ.get(
            "WEB_SLAM_CMD",
            "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true"
        )

        if self.slam_process is not None and self.slam_process.poll() is None:
            self.publish_status("MAPPING_ALREADY_RUNNING")
            self.publish_mapping_status("MAPPING_ALREADY_RUNNING")
            return

        self.slam_process = self.run_process(slam_cmd, "SLAM Mapping")
        time.sleep(1.0)
        self.publish_status("MAPPING_STARTED")
        self.publish_mapping_status("MAPPING_STARTED")

    def save_map(self, requested_name: str = ""):
        name = safe_name(requested_name)
        output_base = self.map_dir / name

        save_cmd = os.environ.get(
            "WEB_SAVE_MAP_CMD",
            f"ros2 run nav2_map_server map_saver_cli -f {output_base}"
        )
        save_cmd = save_cmd.format(map_path=str(output_base), map_name=name)

        self.publish_status(f"MAP_SAVING:{name}")
        self.publish_mapping_status(f"MAP_SAVING:{name}")
        self.get_logger().info(f"Saving map: {save_cmd}")

        result = subprocess.run(
            split_command(save_cmd),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )

        if result.returncode == 0:
            self.current_map_name = name
            self.current_map_path = self.map_dir / f"{name}.yaml"
            self.publish_status(f"MAP_SAVED:{name}")
            self.publish_mapping_status(f"MAP_SAVED:{name}")
            self.publish_map_list()
        else:
            self.get_logger().error(result.stdout)
            self.get_logger().error(result.stderr)
            self.publish_status(f"MAP_SAVE_FAILED:{name}")
            self.publish_mapping_status(f"MAP_SAVE_FAILED:{name}")

    def map_yaml_from_name(self, name: str) -> Optional[Path]:
        cleaned = safe_name(name)
        candidate = self.map_dir / f"{cleaned}.yaml"
        if candidate.exists():
            return candidate
        return None

    def load_map(self, name: str):
        yaml_path = self.map_yaml_from_name(name)
        if yaml_path is None:
            self.publish_status(f"LOAD_MAP_FAILED:{name}")
            return

        # An toàn: không cho đổi map khi Nav2 đang chạy/robot đang điều hướng.
        # Người dùng phải bấm Stop Navigation trước, tránh lỗi TF/costmap/AMCL/goal cũ.
        if self.is_navigation_running():
            self.publish_status("LOAD_MAP_BLOCKED:STOP_NAVIGATION_FIRST")
            return

        # Load map để xem trên web: tắt SLAM/map preview cũ rồi mở riêng map_server.
        self.stop_mapping()
        self.stop_map_preview()

        self.current_map_name = yaml_path.stem
        self.current_map_path = yaml_path

        # Có thể chỉnh bằng biến môi trường nếu cần.
        map_server_cmd = os.environ.get(
            "WEB_MAP_SERVER_CMD",
            "ros2 run nav2_map_server map_server --ros-args "
            "-p yaml_filename:={map_path} "
            "-p use_sim_time:=true "
            "-r __node:=map_server"
        ).format(map_path=str(yaml_path), map_name=yaml_path.stem)

        lifecycle_cmd = os.environ.get(
            "WEB_MAP_LIFECYCLE_CMD",
            "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args "
            "-p use_sim_time:=true "
            "-p autostart:=true "
            "-p node_names:=\"['map_server']\" "
            "-r __node:=lifecycle_manager_map_preview"
        )

        self.map_server_process = self.run_process(map_server_cmd, "Map Server Preview")
        time.sleep(1.0)
        self.map_lifecycle_process = self.run_process(lifecycle_cmd, "Map Lifecycle Preview")
        time.sleep(1.0)

        self.publish_status(f"MAP_LOADED:{yaml_path.stem}")

    def start_navigation(self, maybe_name: str = ""):
        # Navigation độc lập: tắt SLAM và map preview trước.
        self.stop_mapping()
        self.stop_map_preview()

        if maybe_name.strip():
            yaml_path = self.map_yaml_from_name(maybe_name.strip())
            if yaml_path is None:
                self.publish_status(f"NAVIGATION_FAILED:MAP_NOT_FOUND:{maybe_name.strip()}")
                return
            self.current_map_name = yaml_path.stem
            self.current_map_path = yaml_path

        if self.current_map_path is None:
            maps = sorted(self.map_dir.glob("*.yaml"))
            if maps:
                self.current_map_path = maps[0]
                self.current_map_name = maps[0].stem

        if self.current_map_path is None:
            self.publish_status("NAVIGATION_FAILED:NO_MAP_SELECTED")
            return

        nav_cmd_template = os.environ.get(
            "WEB_NAV_CMD",
            "ros2 launch mobile nav2.launch.py map:={map_path} use_sim_time:=true"
        )
        nav_cmd = nav_cmd_template.format(
            map_path=str(self.current_map_path),
            map_name=self.current_map_name or ""
        )

        if self.nav_process is not None and self.nav_process.poll() is None:
            self.publish_status(f"NAVIGATION_ALREADY_RUNNING:{self.current_map_name}")
            return

        self.nav_process = self.run_process(nav_cmd, "Navigation")
        time.sleep(1.0)
        self.publish_status(f"NAVIGATION_STARTED:{self.current_map_name}")

    def on_system_command(self, msg: String):
        data = msg.data.strip()
        self.get_logger().info(f"Received command: {data}")

        if not data:
            return

        if ":" in data:
            command, value = data.split(":", 1)
            command = command.strip().upper()
            value = value.strip()
        else:
            command = data.strip().upper()
            value = ""

        if command == "START_MAPPING":
            self.start_mapping()
        elif command == "STOP_MAPPING":
            self.stop_mapping()
        elif command == "SAVE_MAP":
            self.save_map(value)
        elif command == "LIST_MAPS":
            self.publish_map_list()
        elif command == "LOAD_MAP":
            self.load_map(value)
        elif command in ("START_NAVIGATION", "START_LOCALIZATION"):
            self.start_navigation(value)
        elif command == "STOP_NAVIGATION":
            self.stop_navigation()
            self.stop_map_preview()
        else:
            self.publish_status(f"UNKNOWN_COMMAND:{data}")

    def destroy_node(self):
        self.stop_mapping()
        self.stop_navigation()
        self.stop_map_preview()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebSystemManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
