#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
import psutil
import json
import time
import math

# Giới hạn tốc độ cấu hình trong nav2_params.yaml
MAX_LINEAR_MPS   = 0.26   # max_vel_x
MAX_ANGULAR_RPS  = 2.0    # max_vel_theta


class SystemStatsPublisher(Node):
    def __init__(self):
        super().__init__('system_stats_publisher')

        # Publisher hệ thống
        self.pub = self.create_publisher(String, '/system_stats', 10)

        # Publisher tốc độ / giới hạn
        self.pub_speed = self.create_publisher(String, '/speed_limit', 10)

        # Publisher trạng thái mode hiện tại → web
        self.pub_mode = self.create_publisher(String, '/robot_mode_status', 10)

        # Publisher forward teleop → robot (chỉ dùng khi mode=manual)
        self.pub_cmdvel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber nhận tốc độ thực tế từ EKF odometry
        self.create_subscription(Odometry, '/odom_ekf', self._odom_cb, 10)

        # Subscriber nhận lệnh tốc độ được gửi xuống robot
        self.create_subscription(Twist, '/cmd_vel', self._cmdvel_cb, 10)

        # Subscriber nhận lệnh chuyển mode từ web ("auto" hoặc "manual")
        self.create_subscription(String, '/robot_mode', self._mode_cb, 10)

        # Subscriber nhận teleop từ web (chỉ forward khi mode=manual)
        self.create_subscription(Twist, '/cmd_vel_web', self._cmdvel_web_cb, 10)

        # Action client để cancel Nav2 goal khi chuyển sang manual
        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._nav2_goal_handle = None

        self.timer = self.create_timer(1.0, self.publish_stats)
        self._net_last = psutil.net_io_counters()
        self._net_time = time.time()

        # State tốc độ hiện tại
        self._actual_linear  = 0.0
        self._actual_angular = 0.0
        self._cmd_linear     = 0.0
        self._cmd_angular    = 0.0

        # Mode mặc định là auto
        self._mode = 'auto'

        self.get_logger().info('System stats publisher started — mode: AUTO')

    def _odom_cb(self, msg: Odometry):
        self._actual_linear  = msg.twist.twist.linear.x
        self._actual_angular = msg.twist.twist.angular.z

    def _cmdvel_cb(self, msg: Twist):
        self._cmd_linear  = msg.linear.x
        self._cmd_angular = msg.angular.z

    def _mode_cb(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in ('auto', 'manual'):
            self.get_logger().warn(f'Mode không hợp lệ: {msg.data}')
            return
        if new_mode == self._mode:
            return

        self._mode = new_mode
        self.get_logger().info(f'Chuyển mode → {self._mode.upper()}')

        if self._mode == 'manual':
            # Dừng Nav2 để tránh xung đột cmd_vel
            self._cancel_nav2_goal()
            # Dừng robot ngay lập tức
            stop = Twist()
            self.pub_cmdvel.publish(stop)
        else:
            # Chuyển về auto: dừng robot, Nav2 sẽ tiếp quản
            stop = Twist()
            self.pub_cmdvel.publish(stop)

        self._publish_mode_status()

    def _cancel_nav2_goal(self):
        if self._nav2_goal_handle is not None:
            try:
                self._nav2_goal_handle.cancel_goal_async()
                self.get_logger().info('Đã cancel Nav2 goal')
            except Exception as e:
                self.get_logger().warn(f'Cancel Nav2 goal thất bại: {e}')
            self._nav2_goal_handle = None

    def _cmdvel_web_cb(self, msg: Twist):
        if self._mode == 'manual':
            self.pub_cmdvel.publish(msg)

    def _publish_mode_status(self):
        data = {
            'timestamp': time.time(),
            'mode': self._mode,
            'description': 'Điều hướng tự động (Nav2)' if self._mode == 'auto' else 'Điều khiển thủ công (Teleop)',
        }
        m = String()
        m.data = json.dumps(data)
        self.pub_mode.publish(m)

    def publish_speed_limit(self):
        lin_pct = round(abs(self._actual_linear)  / MAX_LINEAR_MPS  * 100, 1)
        ang_pct = round(abs(self._actual_angular) / MAX_ANGULAR_RPS * 100, 1)

        if abs(self._actual_linear) < 0.01 and abs(self._actual_angular) < 0.01:
            status = 'stopped'
        elif lin_pct >= 95 or ang_pct >= 95:
            status = 'at_limit'
        elif abs(self._actual_linear) < 0.02:
            status = 'rotating'
        else:
            status = 'moving'

        data = {
            'timestamp': time.time(),
            'limits': {
                'max_linear_mps':  MAX_LINEAR_MPS,
                'max_angular_rps': MAX_ANGULAR_RPS,
            },
            'actual': {
                'linear_mps':      round(self._actual_linear, 3),
                'angular_rps':     round(self._actual_angular, 3),
                'linear_percent':  min(lin_pct, 100.0),
                'angular_percent': min(ang_pct, 100.0),
            },
            'commanded': {
                'linear_mps':  round(self._cmd_linear, 3),
                'angular_rps': round(self._cmd_angular, 3),
            },
            'status': status,
        }

        msg = String()
        msg.data = json.dumps(data)
        self.pub_speed.publish(msg)

    def publish_stats(self):
        # CPU
        cpu_percent = psutil.cpu_percent(interval=None)
        cpu_freq = psutil.cpu_freq()
        cpu_count = psutil.cpu_count()

        # RAM
        ram = psutil.virtual_memory()

        # Disk
        disk = psutil.disk_usage('/')

        # Network (tính tốc độ in/out kể từ lần trước)
        net_now = psutil.net_io_counters()
        now = time.time()
        dt = now - self._net_time if now - self._net_time > 0 else 1.0
        net_in_kbps = (net_now.bytes_recv - self._net_last.bytes_recv) / dt / 1024
        net_out_kbps = (net_now.bytes_sent - self._net_last.bytes_sent) / dt / 1024
        self._net_last = net_now
        self._net_time = now

        # Nhiệt độ (nếu có sensor)
        temperature = None
        try:
            temps = psutil.sensors_temperatures()
            if temps:
                for key in ['coretemp', 'cpu_thermal', 'k10temp', 'acpitz']:
                    if key in temps and temps[key]:
                        temperature = temps[key][0].current
                        break
        except (AttributeError, Exception):
            pass

        # Uptime
        uptime_sec = int(time.time() - psutil.boot_time())
        hours, rem = divmod(uptime_sec, 3600)
        minutes, seconds = divmod(rem, 60)

        stats = {
            'timestamp': time.time(),
            'cpu': {
                'percent': cpu_percent,
                'freq_mhz': round(cpu_freq.current, 1) if cpu_freq else None,
                'cores': cpu_count,
            },
            'ram': {
                'percent': ram.percent,
                'used_mb': round(ram.used / 1024 / 1024, 1),
                'total_mb': round(ram.total / 1024 / 1024, 1),
            },
            'disk': {
                'percent': disk.percent,
                'used_gb': round(disk.used / 1024 / 1024 / 1024, 2),
                'total_gb': round(disk.total / 1024 / 1024 / 1024, 2),
            },
            'network': {
                'in_kbps': round(net_in_kbps, 1),
                'out_kbps': round(net_out_kbps, 1),
                'total_recv_mb': round(net_now.bytes_recv / 1024 / 1024, 1),
                'total_sent_mb': round(net_now.bytes_sent / 1024 / 1024, 1),
            },
            'temperature_c': round(temperature, 1) if temperature else None,
            'uptime': f'{hours:02d}:{minutes:02d}:{seconds:02d}',
        }

        msg = String()
        msg.data = json.dumps(stats)
        self.pub.publish(msg)

        self.publish_speed_limit()
        self._publish_mode_status()


def main():
    rclpy.init()
    node = SystemStatsPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
