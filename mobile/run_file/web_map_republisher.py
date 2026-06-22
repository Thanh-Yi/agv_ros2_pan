#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)
from nav_msgs.msg import OccupancyGrid


class WebMapRepublisher(Node):
    def __init__(self):
        super().__init__("web_map_republisher")

        # Subscribe /map với QoS giống map_server/slam_toolbox:
        # RELIABLE + TRANSIENT_LOCAL để lấy được cả map tĩnh và map đang SLAM.
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Publish /web_map cũng dùng RELIABLE + TRANSIENT_LOCAL.
        # Như vậy ros2 topic echo, rosbridge và web đều dễ nhận hơn.
        web_map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.latest_map = None
        self.map_count = 0

        self.sub = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.on_map,
            map_qos,
        )

        self.pub = self.create_publisher(
            OccupancyGrid,
            "/web_map",
            web_map_qos,
        )

        self.timer = self.create_timer(1.0, self.publish_latest)

        self.get_logger().info("Web map republisher ready: /map -> /web_map")
        self.get_logger().info("QoS: RELIABLE + TRANSIENT_LOCAL")

    def on_map(self, msg):
        self.latest_map = msg
        self.map_count += 1
        self.pub.publish(msg)

        if self.map_count <= 5 or self.map_count % 20 == 0:
            self.get_logger().info(
                f"Republished map #{self.map_count}: "
                f"width={msg.info.width}, height={msg.info.height}, "
                f"resolution={msg.info.resolution}"
            )

    def publish_latest(self):
        if self.latest_map is not None:
            self.pub.publish(self.latest_map)


def main(args=None):
    rclpy.init(args=args)
    node = WebMapRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
