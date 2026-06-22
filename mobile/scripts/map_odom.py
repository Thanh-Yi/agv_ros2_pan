#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster

from tf_transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    translation_matrix,
    inverse_matrix,
    concatenate_matrices
)


class MapOdomCorrector(Node):
    def __init__(self):
        super().__init__('map_odom_corrector')

        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.pose_topic = self.declare_parameter(
            'pose_topic',
            '/rtabmap/localization_pose'
        ).value

        # Tần số publish TF map -> odom
        self.publish_rate = self.declare_parameter('publish_rate', 50.0).value

        # Stamp TF hơi vượt trước hiện tại một chút để Nav2 không báo TF quá cũ
        self.future_offset = self.declare_parameter('future_offset', 0.05).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Lưu transform map -> odom mới nhất
        self.last_map_odom = None

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_callback,
            10
        )

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info(
            f'Listening {self.pose_topic}, publishing {self.map_frame}->{self.odom_frame} at {self.publish_rate} Hz'
        )

    def pose_to_matrix(self, pose):
        q = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        ]

        t = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]

        mat_t = translation_matrix(t)
        mat_q = quaternion_matrix(q)

        return concatenate_matrices(mat_t, mat_q)

    def transform_to_matrix(self, transform):
        q = [
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        ]

        t = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        ]

        mat_t = translation_matrix(t)
        mat_q = quaternion_matrix(q)

        return concatenate_matrices(mat_t, mat_q)

    def matrix_to_transform(self, mat):
        trans = TransformStamped()

        # Quan trọng:
        # Không dùng msg.header.stamp của RTAB vì thường bị cũ.
        # Dùng thời gian hiện tại + 0.05s để Nav2 luôn có TF mới.
        now = self.get_clock().now()
        future_time = now + Duration(seconds=float(self.future_offset))

        trans.header.stamp = future_time.to_msg()
        trans.header.frame_id = self.map_frame
        trans.child_frame_id = self.odom_frame

        trans.transform.translation.x = float(mat[0, 3])
        trans.transform.translation.y = float(mat[1, 3])
        trans.transform.translation.z = float(mat[2, 3])

        q = quaternion_from_matrix(mat)
        trans.transform.rotation.x = float(q[0])
        trans.transform.rotation.y = float(q[1])
        trans.transform.rotation.z = float(q[2])
        trans.transform.rotation.w = float(q[3])

        return trans

    def pose_callback(self, msg):
        try:
            # RTAB pose: map -> base_footprint
            t_map_base = self.pose_to_matrix(msg.pose.pose)

            # EKF TF: odom -> base_footprint
            # Lấy TF mới nhất để tránh lỗi timestamp
            odom_base_tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            t_odom_base = self.transform_to_matrix(odom_base_tf.transform)

            # Công thức:
            # map -> odom = map -> base × inverse(odom -> base)
            self.last_map_odom = concatenate_matrices(
                t_map_base,
                inverse_matrix(t_odom_base)
            )

        except Exception as e:
            self.get_logger().warn(f'Cannot compute map->odom: {e}')

    def timer_callback(self):
        if self.last_map_odom is None:
            return

        tf_msg = self.matrix_to_transform(self.last_map_odom)
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomCorrector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()