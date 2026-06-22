from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Đường dẫn tới file cấu hình EKF
    ekf_config = PathJoinSubstitution([
        FindPackageShare('mobile'),   # đổi 'mobile' thành tên package của bạn nếu khác
        'config',
        'ekf.yaml'
    ])

    # --- Node EKF (robot_localization) ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': True}
        ],
        remappings=[
            ('/odometry/filtered', '/odom_ekf'),
        ]
    )

    return LaunchDescription([
        ekf_node
    ])