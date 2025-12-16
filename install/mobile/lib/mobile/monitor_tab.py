import math
import rclpy
from nav_msgs.msg import Odometry
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QGridLayout
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
from collections import deque

# Thiết lập giao diện sáng (white theme)
pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')


class MonitorTab(QWidget):
    def __init__(self, ros_wrapper, control_tab):
        super().__init__()
        self.ros = ros_wrapper
        self.control_tab = control_tab
        self.control_tab.goal_reached_signal.connect(self.on_goal_reached)

        # Dữ liệu ROS
        self.last_odom = None
        self.yaw_points = []

        # ===== MAIN LAYOUT =====
        main_layout = QVBoxLayout(self)
        grid = QGridLayout()
        main_layout.addLayout(grid)
        self.setLayout(main_layout)

        # ===== PATH PLOT =====
        self.path_plot = pg.PlotWidget(title="Robot Path (X-Y)")
        self.path_plot.setLabel('bottom', 'X [m]')
        self.path_plot.setLabel('left', 'Y [m]')
        self.path_plot.showGrid(x=True, y=True)
        self.path_curve = self.path_plot.plot(pen=pg.mkPen('b', width=2))
        self.current_point = self.path_plot.plot(symbol='o', symbolBrush='r', symbolSize=10)
        self.x_data, self.y_data = [], []

        # ===== VELOCITY PLOT =====
        self.vel_plot = pg.PlotWidget(title="Velocity over Time")
        self.vel_plot.setLabel('bottom', 'Time [s]')
        self.vel_plot.setLabel('left', 'Velocity')
        self.vel_plot.showGrid(x=True, y=True)
        self.vel_plot.addLegend()
        self.vel_plot.setYRange(-1, 1)
        self.lin_vel_curve = self.vel_plot.plot(pen=pg.mkPen('b', width=2), name="Linear (m/s)")
        self.ang_vel_curve = self.vel_plot.plot(pen=pg.mkPen('orange', width=2), name="Angular (rad/s)")
        self.time_data = deque(maxlen=200)
        self.lin_vel_data = deque(maxlen=200)
        self.ang_vel_data = deque(maxlen=200)
        self.start_time = None

        # ===== POSITION ERROR PLOT =====
        self.err_plot = pg.PlotWidget(title="Position Error at Goals")
        self.err_plot.setLabel('bottom', 'Error X [m]')
        self.err_plot.setLabel('left', 'Error Y [m]')
        self.err_plot.showGrid(x=True, y=True)

        # ===== YAW ERROR PLOT =====
        self.yaw_plot = pg.PlotWidget(title="Yaw Desired vs Actual")
        self.yaw_plot.setLabel('bottom', 'Yaw Desired [rad]')
        self.yaw_plot.setLabel('left', 'Yaw Actual [rad]')
        self.yaw_plot.showGrid(x=True, y=True)

        # ---- Add plots to grid (2x2) ----
        grid.addWidget(self.path_plot, 0, 0)
        grid.addWidget(self.vel_plot, 0, 1)
        grid.addWidget(self.err_plot, 1, 0)
        grid.addWidget(self.yaw_plot, 1, 1)

        # ---- ROS Subscriptions ----
        self.ros.node.create_subscription(Odometry, "/odometry/filtered", self.odom_callback, 10)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(100)

    def odom_callback(self, msg):
        """Lưu dữ liệu Odom"""
        self.last_odom = msg
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

        self.ros.last_x = x
        self.ros.last_y = y
        self.ros.last_yaw = yaw

        if self.start_time is None:
            self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - self.start_time

        self.x_data.append(x)
        self.y_data.append(y)
        self.time_data.append(t)
        self.lin_vel_data.append(msg.twist.twist.linear.x)
        self.ang_vel_data.append(msg.twist.twist.angular.z)

    def update_plot(self):
        """Cập nhật đồ thị động"""
        if len(self.x_data) > 1:
            self.path_curve.setData(self.x_data, self.y_data)
            self.current_point.setData([self.x_data[-1]], [self.y_data[-1]])
        if len(self.time_data) > 1:
            self.lin_vel_curve.setData(self.time_data, self.lin_vel_data)
            self.ang_vel_curve.setData(self.time_data, self.ang_vel_data)

    def on_goal_reached(self, name, x_err, y_err, yaw_target):
        """Vẽ dữ liệu khi đến đích"""
        # --- Sai số vị trí ---
        self.err_plot.plot([x_err], [y_err], pen=None,
                           symbol='o', symbolBrush='r', symbolSize=10)
        label = pg.TextItem(text=name, anchor=(0.5, -0.3))
        self.err_plot.addItem(label)
        label.setPos(x_err, y_err)

        # --- Sai số yaw ---
        if self.last_odom:
            o = self.last_odom.pose.pose.orientation
            current_yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                                     1.0 - 2.0 * (o.y ** 2 + o.z ** 2))

            # Chấm điểm
            self.yaw_points.append((yaw_target, current_yaw))
            x_vals, y_vals = zip(*self.yaw_points)
            self.yaw_plot.clear()
            self.yaw_plot.plot(x_vals, y_vals, pen=None, symbol='o',
                               symbolBrush='r', symbolSize=10)
            # Thêm tên điểm
            label = pg.TextItem(text=name, anchor=(0.5, -0.3))
            self.yaw_plot.addItem(label)
            label.setPos(yaw_target, current_yaw)
