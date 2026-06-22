import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool, String
from nav2_msgs.action import NavigateToPose


class WebGoalToNav2(Node):
    def __init__(self):
        super().__init__('web_goal_to_nav2')

        self.current_goal_handle = None

        self.goal_sub = self.create_subscription(
            Pose2D,
            '/web_goal_pose',
            self.goal_callback,
            10
        )

        self.cancel_sub = self.create_subscription(
            Bool,
            '/web_cancel_goal',
            self.cancel_callback,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/web_nav_status',
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.get_logger().info('Web Goal To Nav2 node started.')
        self.get_logger().info('Listening: /web_goal_pose')
        self.get_logger().info('Listening: /web_cancel_goal')
        self.get_logger().info('Publishing: /web_nav_status')
        self.get_logger().info('Sending goal to: /navigate_to_pose')

        self.publish_status('IDLE')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f'Navigation Status: {status}')

    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    def goal_callback(self, msg):
        self.get_logger().info(
            f'Received web goal: x={msg.x}, y={msg.y}, theta={msg.theta}'
        )

        self.publish_status('GOAL_SENT')

        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('Nav2 action server /navigate_to_pose is not available.')
            self.publish_status('FAILED')
            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = msg.x
        goal_msg.pose.pose.position.y = msg.y
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = self.yaw_to_quaternion(msg.theta)

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info('Sending goal to Nav2...')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2.')
            self.current_goal_handle = None
            self.publish_status('FAILED')
            return

        self.current_goal_handle = goal_handle

        self.get_logger().info('Goal accepted by Nav2.')
        self.publish_status('MOVING')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        status = result.status

        self.get_logger().info(f'Nav2 goal finished. Status code: {status}')

        # ROS2 action status:
        # 4 = SUCCEEDED
        # 5 = CANCELED
        # 6 = ABORTED / FAILED
        if status == 4:
            self.publish_status('SUCCEEDED')
        elif status == 5:
            self.publish_status('CANCELED')
        else:
            self.publish_status('FAILED')

        self.current_goal_handle = None

    def cancel_callback(self, msg):
        if not msg.data:
            return

        self.get_logger().info('Received cancel request from web.')

        if self.current_goal_handle is None:
            self.get_logger().warn('No active goal to cancel.')
            self.publish_status('IDLE')
            return

        self.publish_status('CANCELING')

        cancel_future = self.current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Nav2 goal cancel request accepted.')
            self.publish_status('CANCELED')
        else:
            self.get_logger().warn('Nav2 goal cancel request rejected.')
            self.publish_status('FAILED')


def main(args=None):
    rclpy.init(args=args)
    node = WebGoalToNav2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
