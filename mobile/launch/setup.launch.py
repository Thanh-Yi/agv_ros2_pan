from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ===== Encoder =====
    encoder = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pan/ros2_ws/src/mobile/read_velocity/encoder.py'
        ],
        output='screen'
    )

    # ===== Send velocity (STM32) =====
    send_velocity = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pan/ros2_ws/src/mobile/read_velocity/send_velocity.py'
        ],
        output='screen'
    )

    # ===== IMU =====
    imu = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pan/ros2_ws/src/mobile/scripts/hwt901b_imu.py'
        ],
        output='screen'
    )

    # ===== LiDAR =====
    lidar = ExecuteProcess(
        cmd=[
            'python3',
            '/home/pan/ros2_ws/src/mobile/scripts/lds50cr.py'
        ],
        output='screen'
    )

    return LaunchDescription([

        encoder,

        # delay để tránh tranh port / tài nguyên
        TimerAction(period=1.0, actions=[send_velocity]),
        TimerAction(period=2.0, actions=[imu]),
        TimerAction(period=3.0, actions=[lidar]),

    ])
