from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    model_file = '/home/ubunturic/ros2_ws/src/mobile/models/obstacle_box.sdf'

    # spawn_static_1 = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'obstacle_static_1', '-file', model_file, '-x', '3', '-y', '0', '-z', '0.35']
    # )

    # spawn_static_2 = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'obstacle_static_2', '-file', model_file, '-x', '0', '-y', '13', '-z', '0.35']
    # )

    spawn_moving = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'obstacle_moving', '-file', model_file, '-x', '0', '-y', '5', '-z', '0.35']
    )

    move_script = Node(
        package='mobile',
        executable='moving_obstacle.py',
        output='screen'
    )

    return LaunchDescription([
        # spawn_static_1,
        # spawn_static_2,
        spawn_moving,
        move_script
    ])
