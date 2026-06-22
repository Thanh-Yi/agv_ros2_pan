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
            # Nếu cần pointcloud thì bật thêm:
            # 'pointcloud.enable': 'true',
        }.items()
    )

    # =========================
    # STATIC TF CAMERA
    # Chỉnh xyz/rpy theo vị trí camera thật trên robot
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

            # database
            'database_path': '/home/pan/rtab_map/map_59.db',

            # =========================
            # CAMERA RGB-D
            # =========================
            'rgb_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',

            'subscribe_rgb': 'true',
            'subscribe_depth': 'true',

            # =========================
            # LIDAR CONFIG
            # Dùng topic LiDAR thật của bạn
            # =========================
            'subscribe_scan': 'true',

            # Nếu LiDAR thật publish /scan thì dùng dòng này:
            'scan_topic': '/scan',

            # Nếu bạn có node filter tạo /scan_filtered thì đổi thành:
            # 'scan_topic': '/scan_filtered',

            # =========================
            # FRAME
            # =========================
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'publish_tf': 'true',
            'wait_for_transform': '1.0',

            # =========================
            # SYNC
            # =========================
            'approx_sync': 'true',
            'queue_size': '30',

            # =========================
            # ODOM
            # =========================
            'visual_odometry': 'false',
            'odom_topic': '/odometry/filtered',

            # =========================
            # DISPLAY
            # =========================
            'rviz': 'false',
            'rtabmap_viz': 'true',

            # =========================
            # RTABMAP ARGS
            # =========================
            'rtabmap_args': (

                # reset map
               # '--delete_db_on_start '
                '--Grid/RayTracing false '
                '--Grid/Scan2dUnknownSpaceFilled false '
                '--Grid/NoiseFilteringRadius 0.03 '
                '--Grid/NoiseFilteringMinNeighbors 1 '
                # =========================
                # GRID MAP 2D
                # Kiểu B: LiDAR tạo map 2D chính
                # Camera vẫn dùng cho feature / loop closure
                # =========================
                '--Grid/FromDepth false '
                '--Grid/3D false '
                '--Grid/RangeMax 5.0 '
                '--Grid/RangeMin 0.3 '
                '--Grid/CellSize 0.05 '



                '--Grid/GroundIsObstacle false '
                '--Grid/MaxGroundHeight 0.1 '
                '--Grid/MaxObstacleHeight 2.0 '
                '--Grid/NormalsSegmentation false '

                # =========================
                # ROBOT MOBILE 2D
                # =========================
                '--Reg/Force3DoF true '

                # Ưu tiên visual registration để giảm loop sai do LiDAR
                '--Reg/Strategy 0 '

                # =========================
                # VISUAL FEATURES
                # =========================
                '--Kp/MaxFeatures 700 '
                '--Vis/MaxFeatures 700 '
                '--Vis/MinInliers 60 '
                '--Kp/DetectorStrategy 6 '

                # =========================
                # LOOP CLOSURE AN TOÀN HƠN
                # =========================
                '--RGBD/ProximityBySpace true '
                '--RGBD/ProximityByTime false '
                '--RGBD/ProximityPathMaxNeighbors 1 '
                '--RGBD/LoopClosureReextractFeatures true '
                '--RGBD/OptimizeFromGraphEnd true '
                '--RGBD/NeighborLinkRefining false '
                '--RGBD/OptimizeMaxError 0.3 '

                # =========================
                # MEMORY MAPPING
                # =========================
                '--Mem/IncrementalMemory true '
                '--Mem/InitWMWithAllNodes true '
            )
        }.items()
    )

    return LaunchDescription([
        camera_static_tf,

        # Chạy camera trước
        realsense,

        # Chờ camera lên topic rồi mới chạy RTAB-Map
        TimerAction(
            period=3.0,
            actions=[rtabmap]
        )
    ])