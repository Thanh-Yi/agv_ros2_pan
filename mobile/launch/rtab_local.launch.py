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

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        launch_arguments={
            'align_depth.enable': 'true',
            # Nếu cần pointcloud:
            # 'pointcloud.enable': 'true',
        }.items()
    )

    # =========================
    # STATIC TF CAMERA
    # Chỉnh xyz/rpy theo vị trí camera thật trên robot
    # Nếu robot_state_publisher/URDF đã có camera_link rồi thì bỏ node này
    # =========================
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

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            # =========================
            # BASIC
            # =========================
            'use_sim_time': 'false',
            'rtabmap': 'true',
            'localization': 'true',

            # Dùng lại database đã mapping thực tế
            'database_path': '/home/pan/rtab_map/map_54.db',

            # =========================
            # FRAME ROBOT
            # =========================
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # RTAB xuất TF map -> odom
            'publish_tf': 'true',
            'wait_for_transform': '1.0',

            # =========================
            # CAMERA RGB-D TOPICS REALSENSE
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

            # Nếu robot thật publish scan gốc:
            'scan_topic': '/scan',

            # Nếu bạn có node lọc scan trên robot thật thì đổi thành:
            # 'scan_topic': '/scan_filtered',

            # =========================
            # SYNC
            # =========================
            'approx_sync': 'true',
            'queue_size': '30',

            # =========================
            # ODOM INPUT
            # Dùng EKF thật
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
            # Localization KHÔNG có --delete_db_on_start
            # =========================
            'rtabmap_args': (
                # =========================
                # MAP 2D / GRID
                # LiDAR là nguồn chính tạo occupancy grid 2D
                # Camera vẫn dùng cho feature / relocalization
                # =========================
                '--Grid/FromDepth false '
                '--Grid/3D false '
                '--Grid/RangeMax 8.0 '
                '--Grid/RangeMin 0.3 '
                '--Grid/CellSize 0.05 '

                # =========================
                # LỌC MẶT ĐẤT / VẬT CẢN
                # =========================
                '--Grid/GroundIsObstacle false '
                '--Grid/MaxGroundHeight 0.15 '
                '--Grid/MaxObstacleHeight 2.0 '
                '--Grid/NormalsSegmentation false '

                # =========================
                # ROBOT MOBILE 2D
                # =========================
                '--Reg/Force3DoF true '

                # Ưu tiên visual registration, tránh LiDAR gây nhận nhầm vị trí
                '--Reg/Strategy 0 '

                # =========================
                # VISUAL FEATURES
                # Local nhẹ hơn mapping một chút để dễ nhận lại vị trí
                # =========================
                '--Kp/MaxFeatures 600 '
                '--Vis/MaxFeatures 600 '
                '--Vis/MinInliers 40 '
                '--Kp/DetectorStrategy 6 '

                # =========================
                # LOOP CLOSURE / RELOCALIZATION
                # =========================
                '--RGBD/ProximityBySpace true '
                '--RGBD/ProximityByTime false '
                '--RGBD/ProximityPathMaxNeighbors 1 '
                '--RGBD/LoopClosureReextractFeatures true '
                '--RGBD/OptimizeFromGraphEnd true '
                '--RGBD/NeighborLinkRefining false '
                '--RGBD/OptimizeMaxError 0.5 '

                # =========================
                # MEMORY / DATABASE LOCALIZATION
                # =========================
                '--Mem/IncrementalMemory false '
                '--Mem/InitWMWithAllNodes true'
            )
        }.items()
    )

    return LaunchDescription([
        camera_static_tf,

        # Chạy camera trước
        realsense,

        # Chờ camera lên topic rồi mới chạy RTAB localization
        TimerAction(
            period=3.0,
            actions=[rtabmap]
        )
    ])