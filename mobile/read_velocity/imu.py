#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import serial
import struct
import rclpy
from rclpy.node import Node
import math
import platform
import serial.tools.list_ports
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from tf_transformations import quaternion_from_euler
import time
import modbus_tk
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu

# 查找 ttyUSB* 设备
def find_ttyUSB():
    print('The default serial port of the imu is /dev/ttyUSB0, if multiple serial port devices are identified, modify the serial port corresponding to the imu in the launch file')
    posts = [port.device for port in serial.tools.list_ports.comports() if 'USB' in port.device]
    print('There are {} {} serial port devices connected to the current PC: {}'.format(len(posts), 'USB', posts))



angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]


if __name__ == "__main__":
    python_version = platform.python_version()[0]

    rclpy.init()
    node = Node("imu")
    node.declare_parameter("port", "/dev/ch34x_imu")
    node.declare_parameter("baud", 115200)

    port = node.get_parameter("port").value
    baudrate = node.get_parameter("baud").value
    print("IMU Type: Modbus Port:%s baud:%d" %(port,baudrate))
    imu_msg = Imu()
    mag_msg = MagneticField()

    # WT901 gyroscope: tin tưởng vừa phải, để wheel encoder vẫn ảnh hưởng chính cho yaw
    imu_msg.angular_velocity_covariance[0] = 1e-3
    imu_msg.angular_velocity_covariance[4] = 1e-3
    imu_msg.angular_velocity_covariance[8] = 1e-3

    # Orientation từ magnetometer không đáng tin trong nhà → đánh dấu -1
    imu_msg.orientation_covariance[0] = -1.0

    # Acceleration không dùng trong EKF
    imu_msg.linear_acceleration_covariance[0] = -1.0

    try:
        wt_imu = serial.Serial(port=port, baudrate=baudrate, timeout=0.5)
        if wt_imu.isOpen():
            node.get_logger().info("\033[32mport open success...\033[0m")
        else:
            wt_imu.open()
            node.get_logger().info("\033[32mport open success...\033[0m")
    except Exception as e:
        print(e)
        node.get_logger().error("\033[31mport open failed\033[0m")
        exit(0)
    else:
        imu_pub = node.create_publisher(Imu, "/imu/data", 50)
        mag_pub = node.create_publisher(MagneticField, "/imu/mag", 50)

        master = modbus_rtu.RtuMaster(wt_imu)
        master.set_timeout(1)
        master.set_verbose(True)
        try:
            while rclpy.ok():            
                
                try:
                    reg = master.execute(80,cst.READ_HOLDING_REGISTERS,52,12)
                except Exception as e:
                    print(e)
                    node.get_logger().info("\033[31mread register time out, please check connection or baundrate set!\033[0m")
                    time.sleep(0.1)
                else:
                    v=[0]*12
                    for i in range(0,12):
                        if (reg[i]>32767):
                            v[i]=reg[i]-65536
                        else:

                            v[i]=reg[i]

                    acceleration = [v[i] / 32768.0 * 16 * 9.8 for i in range(0, 3)]
                    angularVelocity = [v[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3, 6)]
                    magnetometer = v[6:9]
                    angle_degree = [v[i] / 32768.0 * 180 for i in range(9, 12)]
                
                    stamp = node.get_clock().now().to_msg()

                    imu_msg.header.stamp = stamp
                    imu_msg.header.frame_id = "base_link"

                    mag_msg.header.stamp = stamp
                    mag_msg.header.frame_id = "base_link"

                    angle_radian = [angle_degree[i] * math.pi / 180 for i in range(3)]
                    qua = quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

                    imu_msg.orientation.x = float(qua[0])
                    imu_msg.orientation.y = float(qua[1])
                    imu_msg.orientation.z = float(qua[2])
                    imu_msg.orientation.w = float(qua[3])

                    imu_msg.angular_velocity.x = float(angularVelocity[0])
                    imu_msg.angular_velocity.y = float(angularVelocity[1])
                    imu_msg.angular_velocity.z = float(angularVelocity[2])

                    imu_msg.linear_acceleration.x = float(acceleration[0])
                    imu_msg.linear_acceleration.y = float(acceleration[1])
                    imu_msg.linear_acceleration.z = float(acceleration[2])

                    mag_msg.magnetic_field.x = float(magnetometer[0])
                    mag_msg.magnetic_field.y = float(magnetometer[1])
                    mag_msg.magnetic_field.z = float(magnetometer[2])

                    imu_pub.publish(imu_msg)
                    mag_pub.publish(mag_msg)

        except KeyboardInterrupt:
            pass
        finally:
            wt_imu.close()
            node.destroy_node()
            rclpy.shutdown()
           

