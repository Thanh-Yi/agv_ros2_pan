from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    # =========================
    # REALSENSE LAUNCH
    # =========================
    realsense_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py'
    ])

    # =========================
    # RTABMAP LAUNCH
    # =========================
    rtabmap_launch = PathJoinSubstitution([
        FindPackageShare('rtabmap_launch'),
        'launch',
        'rtabmap.launch.py'
    ])

    # =========================
    # REALSENSE CAMERA
    # =========================
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'align_depth.enable': 'true',
           
        }.items()
    )
    camera_static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_to_realsense_tf',
    arguments=[
        '0.33', '0.0', '0.10',
        '0', '0', '0',
        'base_footprint',
        'camera_link'
    ]
)
    # =========================
    # RTAB-MAP LOCALIZATION
    # =========================
    rtabmap_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={

            # =========================
            # BASIC
            # =========================
            'use_sim_time': 'false',
            'rtabmap': 'true',
            'localization': 'true',

            'database_path': '/home/pan/rtab_map/map_54.db',

            # =========================
            # FRAME ROBOT
            # =========================
            'frame_id': 'base_footprint',
            'map_frame_id': 'map',

            # Để rỗng để RTAB lấy odom từ topic /odometry/filtered
            'odom_frame_id': '',

            # RTAB không publish map -> odom
            # map_odom_node sẽ publish map -> odom
            'publish_tf_map': 'false',

            'wait_for_transform': '1.0',

            # =========================
            # CAMERA RGB-D TOPICS
            # =========================
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',

            'subscribe_rgb': 'true',
            'subscribe_depth': 'true',

            # =========================
            # LIDAR CONFIG
            # =========================
            'subscribe_scan': 'true',
            'scan_topic': '/scan',

            # =========================
            # SYNC
            # =========================
            'approx_sync': 'true',
            'queue_size': '30',

            # =========================
            # ODOM INPUT
            # =========================
            'visual_odometry': 'false',
            'odom_topic': '/odometry/filtered',

            # =========================
            # DISPLAY
            # =========================
            'rviz': 'false',
            'rtabmap_viz': 'true',

            # =========================
            # RTAB-MAP PARAMETERS
            # =========================
            'args': (
                '--Grid/FromDepth false '
                '--Grid/3D false '
                '--Grid/RangeMax 8.0 '
                '--Grid/RangeMin 0.3 '
                '--Grid/CellSize 0.05 '

                '--Rtabmap/MaxRepublished 5 '

                '--Grid/GroundIsObstacle false '
                '--Grid/MaxGroundHeight 0.15 '
                '--Grid/MaxObstacleHeight 2.0 '
                '--Grid/NormalsSegmentation false '

                '--Reg/Force3DoF true '
                '--Reg/Strategy 0 '

                '--Kp/MaxFeatures 600 '
                '--Vis/MaxFeatures 600 '
                '--Vis/MinInliers 40 '
                '--Kp/DetectorStrategy 6 '

                '--RGBD/ProximityBySpace true '
                '--RGBD/ProximityByTime false '
                '--RGBD/ProximityPathMaxNeighbors 1 '
                '--RGBD/LoopClosureReextractFeatures true '
                '--RGBD/OptimizeFromGraphEnd true '
                '--RGBD/NeighborLinkRefining false '
                '--RGBD/OptimizeMaxError 0.3 '

                '--Mem/IncrementalMemory false '
                '--Mem/InitWMWithAllNodes true'
            ),
        }.items()
    )

    # =========================
    # MAP -> ODOM CORRECTOR
    # =========================
    map_odom_node = Node(
        package='mobile',
        executable='map_odom',
        name='map_odom_corrector',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_footprint',
            'pose_topic': '/rtabmap/localization_pose',
            'publish_rate': 50.0,
            'future_offset': 0.05
        }]
    )

    return LaunchDescription([
        camera_static_tf,
        # Chạy camera trước
        realsense,

        # Chờ camera lên topic rồi mới chạy RTAB-Map
        TimerAction(
            period=3.0,
            actions=[
                rtabmap_node,
                map_odom_node
            ]
        )
    ])