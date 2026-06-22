#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk

class TeleopGUI(Node):
    def __init__(self):
        super().__init__('teleop_gui')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # GUI
        self.root = tk.Tk()
        self.root.title("Velocity Control")

        # Linear velocity slider
        self.linear_slider = tk.Scale(self.root, from_=-1.3, to=1.3,
                                      resolution=0.01, orient=tk.HORIZONTAL,
                                      label="Linear Velocity (m/s)",
                                      length=400)
        self.linear_slider.pack()

        # Angular velocity slider
        self.angular_slider = tk.Scale(self.root, from_=-2.0, to=2.0,
                                       resolution=0.01, orient=tk.HORIZONTAL,
                                       label="Angular Velocity (rad/s)",
                                       length=400)
        self.angular_slider.pack()

        # Stop button
        stop_btn = tk.Button(self.root, text="STOP", command=self.stop)
        stop_btn.pack()

        # Update loop
        self.update()

    def update(self):
        msg = Twist()
        msg.linear.x = self.linear_slider.get()
        msg.angular.z = self.angular_slider.get()

        self.pub.publish(msg)

        # gọi lại mỗi 100ms (~10Hz)
        self.root.after(100, self.update)

    def stop(self):
        self.linear_slider.set(0.0)
        self.angular_slider.set(0.0)

def main():
    rclpy.init()
    node = TeleopGUI()
    node.root.mainloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()