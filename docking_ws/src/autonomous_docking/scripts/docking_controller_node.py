#!/usr/bin/env python3
"""
Docking Controller Node
-----------------------
Monitors when the robot is physically at the docking pad and publishes
/docking_status  (std_msgs/String: "docked" / "undocked").

Logic:
  - Watches /odom to get current robot position
  - If |robot_pos - dock_pos| < DOCK_RADIUS  →  "docked"
  - Also watches /mission_state to understand context
  - When docked, publishes cmd_vel=0 to ensure the robot stops
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

DOCK_X      = -4.0
DOCK_Y      = -4.0
DOCK_RADIUS = 0.30   # m – if within this radius, robot is "docked"


class DockingControllerNode(Node):

    def __init__(self):
        super().__init__('docking_controller')
        self._robot_x    = 0.0
        self._robot_y    = 0.0
        self._docked     = False
        self._mission    = 'PATROLLING'

        self._pub_status  = self.create_publisher(String, '/docking_status', 10)
        self._pub_cmdvel  = self.create_publisher(Twist,  '/cmd_vel',        10)

        self.create_subscription(Odometry, '/odom',          self._odom_cb,    10)
        self.create_subscription(String,   '/mission_state', self._mission_cb, 10)

        self.create_timer(0.2, self._tick)
        self.get_logger().info('DockingController started')

    def _odom_cb(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _mission_cb(self, msg: String):
        self._mission = msg.data

    def _tick(self):
        dist = math.hypot(self._robot_x - DOCK_X, self._robot_y - DOCK_Y)
        at_dock = (dist < DOCK_RADIUS)

        if at_dock and self._mission in ('DOCKING', 'CHARGING'):
            if not self._docked:
                self.get_logger().info(f'DOCKED  dist={dist:.3f} m')
            self._docked = True
            # Stop the robot
            self._pub_cmdvel.publish(Twist())
        elif not at_dock and self._mission not in ('DOCKING', 'CHARGING'):
            if self._docked:
                self.get_logger().info('UNDOCKED')
            self._docked = False

        s = String()
        s.data = 'docked' if self._docked else 'undocked'
        self._pub_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = DockingControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
