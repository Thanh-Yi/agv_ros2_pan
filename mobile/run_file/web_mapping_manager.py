#!/usr/bin/env python3
"""
web_mapping_manager.py
Nhận lệnh từ web qua /web_mapping_command và xử lý Start/Stop/Save map.

Topic nhận:
  /web_mapping_command  std_msgs/String
    START_MAPPING
    STOP_MAPPING
    SAVE_MAP
    START_LOCALIZATION

Topic phản hồi:
  /web_mapping_status   std_msgs/String

Lưu map mặc định:
  /home/pan/ros2_ws/src/mobile/map/web_map.yaml
  /home/pan/ros2_ws/src/mobile/map/web_map.pgm

Ghi chú:
- Web HTML không tự chạy terminal được, nên cần node này chạy bên ROS2.
- Nếu hệ thống SLAM/map của bạn đã được chạy trong run_web_all.sh hoặc launch Gazebo,
  START_MAPPING chỉ đổi trạng thái. SAVE_MAP vẫn gọi map_saver_cli để lưu /map.
- Nếu muốn nút START_MAPPING tự chạy SLAM, đặt biến môi trường WEB_MAPPING_START_CMD.
  Ví dụ:
  export WEB_MAPPING_START_CMD='ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true'
"""

import os
import shlex
import signal
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WebMappingManager(Node):
    def __init__(self):
        super().__init__('web_mapping_manager')

        self.declare_parameter('map_save_dir', '/home/pan/ros2_ws/src/mobile/map')
        self.declare_parameter('map_name', 'web_map')
        self.declare_parameter('use_sim_time_for_saver', True)
        self.declare_parameter('start_cmd', os.environ.get('WEB_MAPPING_START_CMD', ''))
        self.declare_parameter('stop_cmd', os.environ.get('WEB_MAPPING_STOP_CMD', ''))

        self.map_save_dir = self.get_parameter('map_save_dir').value
        self.map_name = self.get_parameter('map_name').value
        self.use_sim_time_for_saver = bool(self.get_parameter('use_sim_time_for_saver').value)
        self.start_cmd = self.get_parameter('start_cmd').value
        self.stop_cmd = self.get_parameter('stop_cmd').value

        self.mapping_process = None
        self.is_mapping = False

        self.status_pub = self.create_publisher(String, '/web_mapping_status', 10)
        self.cmd_sub = self.create_subscription(
            String,
            '/web_mapping_command',
            self.command_callback,
            10
        )

        self.publish_status('READY')
        self.get_logger().info('Web Mapping Manager is ready.')
        self.get_logger().info(f'Map save path: {self.map_prefix()}')
        if self.start_cmd:
            self.get_logger().info(f'START command: {self.start_cmd}')
        else:
            self.get_logger().warn('No WEB_MAPPING_START_CMD set. START_MAPPING will not launch SLAM; it only changes status.')

    def map_prefix(self):
        return str(Path(self.map_save_dir).expanduser() / self.map_name)

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def command_callback(self, msg):
        command = msg.data.strip().upper()
        self.get_logger().info(f'Received command: {command}')

        if command == 'START_MAPPING':
            self.start_mapping()
        elif command == 'STOP_MAPPING':
            self.stop_mapping()
        elif command == 'SAVE_MAP':
            self.save_map()
        elif command == 'START_LOCALIZATION':
            self.start_localization()
        else:
            self.publish_status(f'UNKNOWN_COMMAND: {command}')

    def start_mapping(self):
        self.is_mapping = True

        if self.mapping_process is not None and self.mapping_process.poll() is None:
            self.publish_status('MAPPING_STARTED: already running')
            return

        if not self.start_cmd:
            self.publish_status('MAPPING_STARTED: using existing /map source')
            return

        try:
            self.mapping_process = subprocess.Popen(
                shlex.split(self.start_cmd),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                preexec_fn=os.setsid
            )
            self.publish_status('MAPPING_STARTED: SLAM command launched')
        except Exception as exc:
            self.mapping_process = None
            self.is_mapping = False
            self.publish_status(f'MAPPING_START_FAILED: {exc}')

    def stop_mapping(self):
        self.is_mapping = False

        if self.stop_cmd:
            try:
                result = subprocess.run(
                    shlex.split(self.stop_cmd),
                    capture_output=True,
                    text=True,
                    timeout=20
                )
                if result.returncode == 0:
                    self.publish_status('MAPPING_STOPPED: stop command done')
                else:
                    self.publish_status(f'MAPPING_STOP_FAILED: {result.stderr.strip()}')
                return
            except Exception as exc:
                self.publish_status(f'MAPPING_STOP_FAILED: {exc}')
                return

        if self.mapping_process is not None and self.mapping_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGINT)
                self.mapping_process.wait(timeout=8)
                self.publish_status('MAPPING_STOPPED: SLAM process stopped')
            except Exception as exc:
                self.publish_status(f'MAPPING_STOP_FAILED: {exc}')
        else:
            self.publish_status('MAPPING_STOPPED: no SLAM process controlled by manager')

    def save_map(self):
        Path(self.map_save_dir).expanduser().mkdir(parents=True, exist_ok=True)
        prefix = self.map_prefix()

        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', prefix]

        if self.use_sim_time_for_saver:
            cmd += ['--ros-args', '-p', 'use_sim_time:=true']

        self.publish_status('MAP_SAVING')
        self.get_logger().info('Running: ' + ' '.join(cmd))

        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=30
            )

            if result.returncode == 0:
                self.publish_status(f'MAP_SAVED: {prefix}.yaml')
            else:
                err = (result.stderr or result.stdout or 'unknown error').strip()
                self.publish_status(f'MAP_SAVE_FAILED: {err}')
        except subprocess.TimeoutExpired:
            self.publish_status('MAP_SAVE_FAILED: timeout while waiting for /map')
        except Exception as exc:
            self.publish_status(f'MAP_SAVE_FAILED: {exc}')

    def start_localization(self):
        self.is_mapping = False
        self.publish_status('MANUAL_CONTROL')

    def destroy_node(self):
        if self.mapping_process is not None and self.mapping_process.poll() is None:
            try:
                os.killpg(os.getpgid(self.mapping_process.pid), signal.SIGINT)
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = WebMappingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
