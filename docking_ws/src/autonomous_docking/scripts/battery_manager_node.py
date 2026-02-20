#!/usr/bin/env python3
"""
Battery Manager Node
--------------------
Simulates a virtual battery that:
  - Starts at 100%
  - Drains at DRAIN_RATE % per second while the robot is moving
  - Drains at IDLE_DRAIN_RATE % per second when stationary
  - Charges at CHARGE_RATE % per second when docked
  - Publishes /battery_level  (std_msgs/Float32, range 0-100)
  - Subscribes /cmd_vel to detect motion
  - Subscribes /docking_status (std_msgs/String: "docked"/"undocked")

Bonus: publishes /remaining_path_energy_ok when predictive scheduling is active.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist


class BatteryManagerNode(Node):
    DRAIN_RATE       = 2.0   # %/s while moving
    IDLE_DRAIN_RATE  = 0.1   # %/s while stationary
    CHARGE_RATE      = 5.0   # %/s while docked
    BATTERY_LOW_THR  = 30.0  # % → trigger dock request
    BATTERY_FULL_THR = 99.0  # % → resume mission after charging
    PUBLISH_HZ       = 2.0

    def __init__(self):
        super().__init__('battery_manager')
        self._level   = 100.0
        self._moving  = False
        self._docked  = False

        self._pub_bat  = self.create_publisher(Float32, '/battery_level', 10)
        self._pub_status = self.create_publisher(String, '/battery_status', 10)

        self.create_subscription(Twist,  '/cmd_vel',        self._cmd_vel_cb,      10)
        self.create_subscription(String, '/docking_status', self._dock_status_cb,  10)

        dt = 1.0 / self.PUBLISH_HZ
        self.create_timer(dt, self._tick)
        self.get_logger().info('BatteryManager started  (100 %)')

    # ── callbacks ──────────────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        self._moving = (abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01)

    def _dock_status_cb(self, msg: String):
        self._docked = (msg.data == 'docked')

    # ── main tick ──────────────────────────────────────────────────────────────
    def _tick(self):
        dt = 1.0 / self.PUBLISH_HZ

        if self._docked:
            self._level = min(100.0, self._level + self.CHARGE_RATE * dt)
        elif self._moving:
            self._level = max(0.0,   self._level - self.DRAIN_RATE   * dt)
        else:
            self._level = max(0.0,   self._level - self.IDLE_DRAIN_RATE * dt)

        bat_msg = Float32()
        bat_msg.data = float(self._level)
        self._pub_bat.publish(bat_msg)

        # Publish human-readable status string
        if self._docked:
            status = f'CHARGING  {self._level:.1f} %'
        elif self._level < self.BATTERY_LOW_THR:
            status = f'LOW  {self._level:.1f} %'
        else:
            status = f'OK  {self._level:.1f} %'
        s = String(); s.data = status
        self._pub_status.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = BatteryManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
