from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_mobile = get_package_share_directory('mobile')

    # Đổi tên file map ở đây nếu map của bạn tên khác
    map_file = os.path.join(pkg_mobile, 'map', 'my_map_rtab42.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file
            }
        ]
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server']
            }
        ]
    )

    return LaunchDescription([
        map_server,
        lifecycle_manager_map
    ])