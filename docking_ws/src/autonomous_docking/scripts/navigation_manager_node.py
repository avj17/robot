#!/usr/bin/env python3
"""
Navigation Manager Node
-----------------------
Bridges /current_goal (PoseStamped) → Nav2 NavigateToPose action.
Reports result back on /nav2_goal_status (std_msgs/String).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class NavigationManagerNode(Node):

    def __init__(self):
        super().__init__('navigation_manager')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._pub_status = self.create_publisher(String, '/nav2_goal_status', 10)
        self.create_subscription(PoseStamped, '/current_goal', self._goal_cb, 10)
        self._current_goal   = None
        self._goal_handle    = None
        self._navigating     = False
        self.get_logger().info('NavigationManager started')

    def _goal_cb(self, msg: PoseStamped):
        # Only send if goal changed significantly
        self._current_goal = msg
        self._send_goal(msg)

    def _send_goal(self, pose: PoseStamped):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            return

        # Cancel any active goal
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.get_logger().info(
            f'Sending goal  x={pose.pose.position.x:.2f}  y={pose.pose.position.y:.2f}')
        self._navigating = True
        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        send_future.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self._navigating = False
            return
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result = future.result().result  # noqa
        status_code = future.result().status
        # status 4 == SUCCEEDED in action_msgs
        if status_code == 4:
            self.get_logger().info('Goal REACHED')
            s = String(); s.data = 'reached'
        else:
            self.get_logger().warn(f'Goal FAILED  status={status_code}')
            s = String(); s.data = 'failed'
        self._pub_status.publish(s)
        self._navigating = False
        self._goal_handle = None

    def _feedback_cb(self, feedback):
        dist = feedback.feedback.distance_remaining
        self.get_logger().debug(f'  distance_remaining={dist:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigationManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
