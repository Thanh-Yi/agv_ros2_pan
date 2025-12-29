#!/usr/bin/env python3
import sys
import math
import yaml
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QVBoxLayout, QGridLayout, QLabel,
    QMessageBox, QHBoxLayout, QScrollArea, QFrame, QTabWidget
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont


# ---------------- Helper functions ----------------
def yaw_to_quaternion(yaw):
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return [qx, qy, qz, qw]


# ---------------- ROS Wrapper ----------------
class RosNodeWrapper:
    def __init__(self):
        self.node = Node('teleop_nav_gui_node')
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def destroy(self):
        try:
            self.node.destroy_node()
        except Exception:
            pass


# ---------------- Control Panel ----------------
class ControlPanel(QWidget):
    goal_reached_signal = pyqtSignal(str, float, float, float)
    def __init__(self, ros_wrapper, goals_yaml_path, history_path):
        super().__init__()
        self.ros = ros_wrapper
        self.goals_yaml_path = goals_yaml_path
        self.history_path = history_path

        self.route_list = []
        self.current_goal_index = 0
        self.route_active = False
        self.goals = []

        layout = QVBoxLayout(self)
        title = QLabel("RICLAB")
        title.setFont(QFont("Arial", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        # Manual control
        teleop_label = QLabel("D-150 Robot")
        teleop_label.setAlignment(Qt.AlignCenter)
        teleop_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(teleop_label)

        grid = QGridLayout()
        layout.addLayout(grid)

        # Movement buttons
        self.btn_w = QPushButton("W")
        self.btn_a = QPushButton("A")
        self.btn_s = QPushButton("S")
        self.btn_d = QPushButton("D")

        for b in (self.btn_w, self.btn_a, self.btn_s, self.btn_d):
            b.setFixedSize(64, 48)
            b.setFont(QFont("Arial", 12, QFont.Bold))
            b.setObjectName("controlButton")

        grid.addWidget(self.btn_w, 0, 1)
        grid.addWidget(self.btn_a, 1, 0)
        grid.addWidget(self.btn_d, 1, 2)
        grid.addWidget(self.btn_s, 2, 1)

        # Movement bindings
        self.btn_w.pressed.connect(lambda: self.publish_cmd(0.25, 0.0))
        self.btn_w.released.connect(self.stop_robot)
        self.btn_s.pressed.connect(lambda: self.publish_cmd(-0.25, 0.0))
        self.btn_s.released.connect(self.stop_robot)
        self.btn_a.pressed.connect(lambda: self.publish_cmd(0.0, 1.0))
        self.btn_a.released.connect(self.stop_robot)
        self.btn_d.pressed.connect(lambda: self.publish_cmd(0.0, -1.0))
        self.btn_d.released.connect(self.stop_robot)

        # Goals section
        layout.addSpacing(10)
        goals_label = QLabel("📍 Predefined Goals")
        goals_label.setAlignment(Qt.AlignCenter)
        goals_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(goals_label)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        layout.addWidget(scroll_area, stretch=1)

        container = QWidget()
        self.goals_layout = QVBoxLayout(container)
        scroll_area.setWidget(container)

        # Route display
        layout.addSpacing(10)
        route_label = QLabel("Current Route")
        route_label.setAlignment(Qt.AlignCenter)
        route_label.setFont(QFont("Arial", 12, QFont.Bold))
        layout.addWidget(route_label)

        self.route_display = QLabel("(Empty)")
        self.route_display.setAlignment(Qt.AlignCenter)
        self.route_display.setObjectName("routeDisplay")
        layout.addWidget(self.route_display)

        # Current target label
        self.current_target_label = QLabel("Current Target: (None)")
        self.current_target_label.setAlignment(Qt.AlignCenter)
        self.current_target_label.setObjectName("currentTarget")
        layout.addWidget(self.current_target_label)

        # Buttons
        btn_layout = QHBoxLayout()
        self.btn_start_route = QPushButton("▶️ Start Route")
        self.btn_cancel_route = QPushButton("🗑️ Cancel Route")

        for b in (self.btn_start_route, self.btn_cancel_route):
            b.setFixedHeight(44)
            b.setFont(QFont("Arial", 12, QFont.Bold))
        btn_layout.addWidget(self.btn_start_route)
        btn_layout.addWidget(self.btn_cancel_route)
        layout.addLayout(btn_layout)

        self.btn_start_route.clicked.connect(self.start_route)
        self.btn_cancel_route.clicked.connect(self.clear_route)

        # Status label
        self.status_label = QLabel("Status: Idle")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)

        # Load goals
        self.load_goals(self.goals_yaml_path)

        # ROS spin timer
        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self.spin_once)
        self.spin_timer.start(10)

    # ---------- ROS ----------
    def spin_once(self):
        try:
            rclpy.spin_once(self.ros.node, timeout_sec=0.0)
        except Exception:
            pass

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.ros.cmd_pub.publish(msg)
        self.status_label.setText(f"Manual: v={v:.2f}, w={w:.2f}")

    def stop_robot(self):
        msg = Twist()
        self.ros.cmd_pub.publish(msg)
        self.status_label.setText("Stopped")

    # ---------- Goals ----------
    def load_goals(self, filepath):
        try:
            with open(filepath, 'r') as f:
                data = yaml.safe_load(f)
            self.goals = data.get('goals', [])
            for g in self.goals:
                name = g.get('name', 'Goal')
                btn = QPushButton(name)
                btn.setFont(QFont("Arial", 11, QFont.Bold))
                btn.setObjectName("goalButton")
                btn.clicked.connect(lambda _, goal=g: self.add_to_route(goal))
                self.goals_layout.addWidget(btn)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load YAML:\n{e}")

    def add_to_route(self, goal):
        self.route_list.append(goal)
        names = [g.get('name', 'Goal') for g in self.route_list]
        self.route_display.setText(" → ".join(names))

    def clear_route(self):
        self.route_list.clear()
        self.route_display.setText("(Empty)")
        self.status_label.setText("Route cleared")

    # ---------- Route ----------
    def start_route(self):
        if not self.route_list:
            QMessageBox.warning(self, "Empty Route", "Please select at least one goal.")
            return
        if not self.ros.nav_client.wait_for_server(timeout_sec=2.0):
            QMessageBox.warning(self, "Nav2", "NavigateToPose server not available.")
            return

        self.save_route_to_history()
        self.current_goal_index = 0
        self.route_active = True
        self.status_label.setText("Starting route...")
        self.send_next_goal()

    def send_next_goal(self):
        if self.current_goal_index >= len(self.route_list):
            self.status_label.setText("✅ Route complete!")
            self.current_target_label.setText("Current Target: (None)")
            self.route_active = False
            return

        goal = self.route_list[self.current_goal_index]
        name = goal.get('name', f"Goal {self.current_goal_index}")
        self.current_target_label.setText(f"Current Target: {name}")
        self.status_label.setText(f"Sending goal {self.current_goal_index+1}/{len(self.route_list)}: {name}")

        pose = PoseStamped()
        pose.header.frame_id = goal.get('frame_id', 'map')
        pose.header.stamp = self.ros.node.get_clock().now().to_msg()
        pos = goal.get('position', [0.0, 0.0, 0.0])
        ori = goal.get('orientation', None)
        yaw = goal.get('yaw', None)

        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2] if len(pos) > 2 else 0.0

        if ori:
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = ori
        elif yaw:
            q = yaw_to_quaternion(yaw)
            pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
        else:
            pose.pose.orientation.w = 1.0

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        send_future = self.ros.nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.status_label.setText("❌ Goal rejected.")
            self.route_active = False
            return
        self.status_label.setText("Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4:
            goal = self.route_list[self.current_goal_index]
            name = goal.get('name', 'Goal')
            self.status_label.setText(f"✅ Reached {name}")

            # Lấy vị trí mong muốn
            gx, gy = goal["position"][:2]
            gq = goal["orientation"]
            # g_yaw = math.atan2(2.0*(gq[3]*gq[2]), 1.0 - 2.0*(gq[2]**2))
            # ✅ Tính yaw mong muốn từ quaternion (đây là yaw_target)
            yaw_target = math.atan2(2.0 * (gq[3] * gq[2] + gq[0] * gq[1]),
                                    1.0 - 2.0 * (gq[1] ** 2 + gq[2] ** 2))

            # Lấy vị trí hiện tại từ monitor_tab (hoặc odometry)
            try:
                cur = self.ros.node.get_parameter_or('current_pose', None)
            except:
                cur = None

            # Nếu có current_pose thì tính sai số, tạm để 0 nếu chưa có
            x_err = getattr(self.ros, 'last_x', 0.0) - gx
            y_err = getattr(self.ros, 'last_y', 0.0) - gy
            # yaw_err = getattr(self.ros, 'last_yaw', 0.0) - g_yaw

            # Phát tín hiệu
            self.goal_reached_signal.emit(name, x_err, y_err, yaw_target)

            self.current_goal_index += 1
            QTimer.singleShot(1000, self.send_next_goal)

        else:
            self.status_label.setText(f"❌ Failed at goal {self.current_goal_index+1}")
            self.route_active = False

    # ---------- History ----------
    def save_route_to_history(self):
        import datetime
        if not self.route_list:
            return
        names = [g.get('name', 'Goal') for g in self.route_list]
        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        route_entry = {'route': names, 'timestamp': now}
        data = {'history': []}
        if os.path.exists(self.history_path):
            try:
                with open(self.history_path, 'r') as f:
                    data = yaml.safe_load(f) or {'history': []}
            except Exception:
                data = {'history': []}
        data['history'].append(route_entry)
        with open(self.history_path, 'w') as f:
            yaml.safe_dump(data, f)


# ---------------- History Tab ----------------
class HistoryTab(QWidget):
    def __init__(self, history_path):
        super().__init__()
        self.history_path = history_path
        layout = QVBoxLayout(self)
        title = QLabel("📜 History")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        layout.addWidget(self.scroll)

        self.container = QWidget()
        self.vbox = QVBoxLayout(self.container)
        self.scroll.setWidget(self.container)

        btn_layout = QHBoxLayout()
        self.btn_refresh = QPushButton("🔄 Refresh")
        self.btn_clear = QPushButton("🗑️ Clear History")
        btn_layout.addWidget(self.btn_refresh)
        btn_layout.addWidget(self.btn_clear)
        layout.addLayout(btn_layout)

        self.btn_refresh.clicked.connect(self.load_history)
        self.btn_clear.clicked.connect(self.clear_history)

        self.load_history()

    def load_history(self):
        for i in reversed(range(self.vbox.count())):
            item = self.vbox.itemAt(i).widget()
            if item:
                item.deleteLater()

        if not os.path.exists(self.history_path):
            lbl = QLabel("No history file found.")
            self.vbox.addWidget(lbl)
            return

        try:
            with open(self.history_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            history = data.get('history', [])
            if not history:
                lbl = QLabel("No routes recorded yet.")
                lbl.setAlignment(Qt.AlignCenter)
                self.vbox.addWidget(lbl)
                return

            text = ""
            for i, entry in enumerate(history, 1):
                route_str = " → ".join(entry.get('route', []))
                time = entry.get('timestamp', '')
                text += f"<b>{i}.</b> {route_str} <br><small style='color:#0055aa;'>📅 {time}</small><br><br>"

            lbl = QLabel(text)
            lbl.setStyleSheet("""
                color: #002b5c;
                background-color: #f0f6ff;
                padding: 10px;
                border-radius: 8px;
                font-size: 13px;
            """)
            lbl.setTextFormat(Qt.RichText)
            lbl.setAlignment(Qt.AlignLeft | Qt.AlignTop)
            self.vbox.addWidget(lbl)

        except Exception as e:
            lbl = QLabel(f"Error reading history: {e}")
            self.vbox.addWidget(lbl)

    def clear_history(self):
        confirm = QMessageBox.question(self, "Confirm", "Clear all history?")
        if confirm == QMessageBox.Yes:
            with open(self.history_path, 'w') as f:
                yaml.safe_dump({'history': []}, f)
            self.load_history()


# ---------------- Main Window ----------------
class MainWindow(QTabWidget):
    def __init__(self, ros_wrapper, goals_path, history_path):
        super().__init__()
        self.setWindowTitle("Mobile Robot GUI")
        self.resize(700, 700)
        self.control_tab = ControlPanel(ros_wrapper, goals_path, history_path)
        self.history_tab = HistoryTab(history_path)
        self.addTab(self.control_tab, "Control Panel")
        self.addTab(self.history_tab, "History")
        # Tab 3 sẽ được thêm sau (Live Monitor)
        # Thêm tab Monitor (giám sát realtime)
        from monitor_tab import MonitorTab  # import file riêng của tab 3
        self.monitor_tab = MonitorTab(ros_wrapper, self.control_tab)
        self.addTab(self.monitor_tab, "Monitor")
        # Kết nối tín hiệu khi đạt mục tiêu
        self.control_tab.goal_reached_signal.connect(self.monitor_tab.on_goal_reached)



# ---------------- Modern Style ----------------
def apply_modern_theme(app):
    app.setStyleSheet("""
        QWidget {
            background-color: #f4f9ff;
            color: #002b5c;
            font-family: 'Segoe UI';
        }
        QLabel {
            color: #003366;
        }
        QPushButton {
            background-color: #e7f0ff;
            border: 1px solid #0078d7;
            border-radius: 8px;
            padding: 6px 12px;
            color: #003366;
            font-weight: 600;
        }
        QPushButton:hover {
            background-color: #cce4ff;
        }
        QPushButton:pressed {
            background-color: #99ccff;
        }
        QPushButton#controlButton {
            background-color: #dceeff;
            border: 2px solid #0078d7;
            color: #002b5c;
            font-size: 14px;
            font-weight: bold;
        }
        QLabel#currentTarget {
            background-color: #fff0f0;
            color: #b22222;
            font-weight: bold;
            border: 1px solid #ffaaaa;
            border-radius: 6px;
            padding: 5px;
        }
        QLabel#routeDisplay {
            background-color: #f0f8ff;
            color: #004b91;
            border: 1px solid #99c4ff;
            border-radius: 6px;
            padding: 6px;
        }
        QTabBar::tab {
            background: #e8f1ff;
            border: 1px solid #0078d7;
            border-radius: 6px;
            padding: 6px 12px;
            min-width: 100px;
            min-height: 28px;
            color: #002b5c;
            font-size: 13px;
            font-weight: 600;
        }
        QTabBar::tab:selected {
            background: #0078d7;
            color: white;
            font-weight: 600;   /* không bold để không phình chữ */
        }
        QTabBar::tab:hover {
            background: #bcd8ff;
        }

    """)


# ---------------- Entry Point ----------------
if __name__ == "__main__":
    rclpy.init()
    ros_wrapper = RosNodeWrapper()

    goals_path = "/home/pan/ros2_ws/src/mobile/scripts/predefined_goals.yaml"
    history_path = "/home/pan/ros2_ws/src/mobile/scripts/history.yaml"

    app = QApplication(sys.argv)
    apply_modern_theme(app)

    window = MainWindow(ros_wrapper, goals_path, history_path)
    window.show()
    app.exec_()

    ros_wrapper.destroy()
    rclpy.shutdown()
