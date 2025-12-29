from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    # ===== Send velocity to STM32 (/cmd_vel -> STM32) =====
    send_velocity = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pan/ros2_ws/src/mobile/read_velocity/gui_encoder.py'
        ],
        output='screen'
    )

    # ===== IMU (WT901 ROS2 NODE - NEW) =====
    imu = Node(
        package='wt901_ros2',
        executable='wt901_node',
        name='wt901_node',
        output='screen'
    )

    # ===== LiDAR (CHƯA DÙNG – COMMENT LẠI) =====
    lidar = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pan/ros2_ws/src/mobile/scripts/lds50cr.py'
        ],
        output='screen'
    )

    return LaunchDescription([

        # encoder chạy trước
        # delay tránh tranh serial
        TimerAction(period=1.0, actions=[send_velocity]),
        TimerAction(period=2.0, actions=[imu]),

        # LiDAR tạm thời KHÔNG chạy
        TimerAction(period=3.0, actions=[lidar]),
    ])
