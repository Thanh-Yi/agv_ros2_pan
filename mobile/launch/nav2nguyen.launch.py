# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():
#     pkg_mobile = get_package_share_directory('mobile')
#     pkg_nav2 = get_package_share_directory('nav2_bringup')

#     params_file = os.path.join(pkg_mobile, 'config', 'nav3.yaml')
#     use_sim_time = LaunchConfiguration('use_sim_time', default='false')

#     nav2_navigation = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
#         ),
#         launch_arguments={
#             'use_sim_time': use_sim_time,
#             'params_file': params_file
#         }.items()
#     )

#     return LaunchDescription([
#         nav2_navigation
#     ])
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():
    pkg_mobile = get_package_share_directory('mobile')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_mobile, 'map', 'map7.yaml')
    # map_file = os.path.join(pkg_mobile, 'map', 'map8.yaml')
    # map_file = os.path.join(pkg_mobile, 'map', 'map_pan_1.yaml')
    # map_file = os.path.join(pkg_mobile, 'map', 'map_pan.yaml')
    params_file = os.path.join(pkg_mobile, 'config', 'nav2_nguyen.yaml')

     #  use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ==== Gọi bringup của Nav2 ====
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            # 'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(nav2_bringup)

    return ld
