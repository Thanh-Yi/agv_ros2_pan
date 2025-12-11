from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mobile')
    urdf_file = os.path.join(pkg_share, 'urdf', 'mobile.urdf')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')  # đường dẫn tới file EKF

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # --- Node EKF (robot_localization) ---
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered'),  # có thể chỉnh nếu muốn rename
        ]
    )

    # --- RViz2 node ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # --- Goal Marker Publisher node ---
    goal_marker_node = Node(
        package='mobile',
        executable='goal_markers.py',
        name='goal_marker_publisher',
        output='screen'
    )

    return LaunchDescription([
        ekf_node,
        rviz_node,
        goal_marker_node,
    ])
