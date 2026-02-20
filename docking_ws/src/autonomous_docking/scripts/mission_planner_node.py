#!/usr/bin/env python3

"""
Mission Planner Node  – A* + Predictive Battery Docking
========================================================
Implements the algorithm from the design notes:

  Given a START waypoint and a GOAL waypoint, compute an A* path through
  the waypoint graph.  At each step from node N → N+1:

      l1 = shortest path cost  N  → N+1
      l2 = shortest path cost  N+1 → nearest dock

      if l1 + l2 < current_battery:
          move to N+1                    ← enough charge
      else:
          go dock, charge to 95%, resume ← not enough, dock first

  When GOAL is reached → publish MISSION_COMPLETE and stop.

States:  INIT → PATROLLING → NEEDS_DOCK → DOCKING → CHARGING → RESUMING → DONE
"""

import math, heapq, yaml, rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


# ─── helpers ──────────────────────────────────────────────────────────────────
def yaw_to_quat(yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)          # x y z w


def euclidean(a, b):
    return math.hypot(a['x'] - b['x'], a['y'] - b['y'])


def build_graph(wps, connect_thresh=8.5):
    """Build adjacency list: connect waypoints closer than connect_thresh metres."""
    keys = list(wps.keys())
    adj = {k: [] for k in keys}
    for i, u in enumerate(keys):
        for v in keys[i+1:]:
            d = euclidean(wps[u], wps[v])
            if d <= connect_thresh:
                adj[u].append((v, d))
                adj[v].append((u, d))
    return adj


def astar(adj, wps, start, goal):
    """A* on the waypoint graph.  Returns list of waypoint names start..goal."""
    open_set = [(0.0, start, [start])]
    g = {start: 0.0}
    while open_set:
        f, cur, path = heapq.heappop(open_set)
        if cur == goal:
            return path
        for nb, cost in adj.get(cur, []):
            ng = g[cur] + cost
            if ng < g.get(nb, float('inf')):
                g[nb] = ng
                h  = euclidean(wps[nb], wps[goal])
                heapq.heappush(open_set, (ng + h, nb, path + [nb]))
    return [start, goal]   # fallback: direct


def path_cost(adj, wps, src, dst):
    """Shortest path cost between two waypoints via Dijkstra."""
    dist = {src: 0.0}
    pq   = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            return d
        if d > dist.get(u, float('inf')):
            continue
        for v, w in adj.get(u, []):
            nd = d + w
            if nd < dist.get(v, float('inf')):
                dist[v] = nd
                heapq.heappush(pq, (nd, v))
    # fallback: straight-line
    return euclidean(wps[src], wps[dst])


def nearest_dock(wps_pos, dock_dict, from_wp):
    best_d, best_k = float('inf'), None
    pos = wps_pos[from_wp]
    for dk, dp in dock_dict.items():
        d = math.hypot(pos['x'] - dp['x'], pos['y'] - dp['y'])
        if d < best_d:
            best_d, best_k = d, dk
    return best_k, best_d


# ─── Node ─────────────────────────────────────────────────────────────────────
class MissionPlannerNode(Node):

    def __init__(self):
        super().__init__('mission_planner')
        self.declare_parameter('waypoints_config', '')

        cfg_path = self.get_parameter('waypoints_config').get_parameter_value().string_value
        self._cfg   = self._load_config(cfg_path)
        self._wps   = self._cfg.get('waypoints', {})
        self._docks = self._cfg.get('docking_stations', {})
        mis         = self._cfg

        self._bat_thr   = float(mis.get('battery_threshold',   30.0))
        self._bat_full  = float(mis.get('battery_dock_return', 99.0))
        self._start_wp  = mis.get('start_waypoint', 'WP2')
        self._goal_wp   = mis.get('goal_waypoint',  'WP8')

        # Build graph
        self._adj  = build_graph(self._wps)
        # Compute A* path start→goal
        self._path = astar(self._adj, self._wps, self._start_wp, self._goal_wp)
        self.get_logger().info(f'A* path: {" → ".join(self._path)}')

        self._state        = 'INIT'
        self._step         = 0          # current index in self._path
        self._battery      = 100.0
        self._goal_reached = False      # feedback from nav manager
        self._robot_x      = 0.0
        self._robot_y      = 0.0
        self._dock_name    = None       # dock we are heading to
        self._resume_step  = 0

        # Publishers
        self._pub_state = self.create_publisher(String,      '/mission_state', 10)
        self._pub_goal  = self.create_publisher(PoseStamped, '/current_goal',  10)
        self._pub_info  = self.create_publisher(String,      '/mission_info',  10)

        # Subscribers
        self.create_subscription(Float32,  '/battery_level',   self._bat_cb,  10)
        self.create_subscription(String,   '/docking_status',  self._dock_cb, 10)
        self.create_subscription(String,   '/nav2_goal_status',self._goal_cb, 10)
        self.create_subscription(Odometry, '/odom',            self._odom_cb, 10)

        self.create_timer(1.0, self._tick)
        self.get_logger().info(f'MissionPlanner ready  start={self._start_wp}  goal={self._goal_wp}')

    # ── config loader ─────────────────────────────────────────────────────────
    def _load_config(self, path):
        if not path:
            self.get_logger().warn('No waypoints_config path given, using defaults')
            return {}
        try:
            with open(path) as f:
                raw = yaml.safe_load(f)
            return raw.get('autonomous_docking', {}).get('ros__parameters', {})
        except Exception as e:
            self.get_logger().error(f'Config load failed: {e}')
            return {}

    # ── callbacks ─────────────────────────────────────────────────────────────
    def _bat_cb(self, msg):
        self._battery = msg.data

    def _dock_cb(self, msg):
        if 'CHARGED' in msg.data or 'COMPLETE' in msg.data:
            if self._state == 'CHARGING':
                self._state = 'RESUMING'
                self.get_logger().info('Charged! Resuming mission...')

    def _goal_cb(self, msg):
        if 'REACHED' in msg.data or 'SUCCEEDED' in msg.data:
            self._goal_reached = True

    def _odom_cb(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    # ── main tick ─────────────────────────────────────────────────────────────
    def _tick(self):
        if self._state == 'INIT':
            self._state = 'PATROLLING'
            self._step  = 0
            self._goal_reached = False
            self._send_waypoint(self._path[self._step])
            return

        if self._state == 'DONE':
            self._pub_state.publish(self._str('MISSION_COMPLETE'))
            return

        if self._state == 'PATROLLING':
            if not self._goal_reached:
                return          # still travelling

            # arrived at path[step]
            cur_wp = self._path[self._step]
            self.get_logger().info(f'Arrived at {cur_wp}  battery={self._battery:.1f}%')
            self._goal_reached = False

            if self._step + 1 >= len(self._path):
                # ── GOAL REACHED ─────────────────────────────────────────
                self._state = 'DONE'
                self.get_logger().info(f'GOAL {self._goal_wp} reached! Mission complete.')
                self._pub_info.publish(self._str(f'GOAL_REACHED:{self._goal_wp}'))
                return

            # ── Battery check: l1 + l2 < current_battery ? ───────────────
            next_wp  = self._path[self._step + 1]
            l1 = path_cost(self._adj, self._wps, cur_wp, next_wp)
            dock_name, _  = nearest_dock(self._wps, self._docks, next_wp)
            l2 = path_cost(self._adj, self._wps, next_wp, 'NEAR_DOCK')

            # Use straight-line to dock from next_wp
            if dock_name:
                nwp  = self._wps[next_wp]
                dpos = self._docks[dock_name]
                l2   = math.hypot(nwp['x']-dpos['x'], nwp['y']-dpos['y'])

            # Convert metres→battery% (1 m ≈ 0.5% drain, rough model)
            DRAIN_PER_M = 0.5
            l1_bat = l1 * DRAIN_PER_M
            l2_bat = l2 * DRAIN_PER_M

            info = (f'Step {self._step}: {cur_wp}→{next_wp}  '
                    f'l1={l1:.1f}m({l1_bat:.1f}%)  '
                    f'l2={l2:.1f}m({l2_bat:.1f}%)  '
                    f'battery={self._battery:.1f}%')
            self.get_logger().info(info)
            self._pub_info.publish(self._str(info))

            if l1_bat + l2_bat < self._battery:
                # ── Enough charge → go to next waypoint ──────────────────
                self._step += 1
                self._send_waypoint(next_wp)
            else:
                # ── Not enough → dock first ───────────────────────────────
                self.get_logger().warn(
                    f'Battery {self._battery:.1f}% insufficient for '
                    f'{cur_wp}→{next_wp}→dock ({l1_bat+l2_bat:.1f}% needed). DOCKING.')
                self._resume_step = self._step + 1
                self._dock_name   = dock_name
                self._state       = 'NEEDS_DOCK'
            return

        if self._state == 'NEEDS_DOCK':
            if self._dock_name and self._dock_name in self._docks:
                self._state = 'DOCKING'
                self._send_dock(self._dock_name)
            return

        if self._state == 'DOCKING':
            if self._goal_reached:
                self._goal_reached = False
                self._state = 'CHARGING'
                self.get_logger().info(f'Docked at {self._dock_name}. Charging...')
                self._pub_info.publish(self._str(f'DOCKED:{self._dock_name}'))
            return

        if self._state == 'CHARGING':
            # wait for dock_cb to flip to RESUMING
            return

        if self._state == 'RESUMING':
            self._step  = self._resume_step
            self._state = 'PATROLLING'
            self._goal_reached = False
            self._send_waypoint(self._path[self._step])
            return

    # ── senders ───────────────────────────────────────────────────────────────
    def _send_waypoint(self, wp_name):
        wp = self._wps.get(wp_name)
        if not wp:
            self.get_logger().error(f'Unknown waypoint: {wp_name}')
            return
        self.get_logger().info(f'Sending goal → {wp_name}  ({wp["x"]}, {wp["y"]})')
        self._pub_state.publish(self._str(f'PATROLLING:{wp_name}'))
        self._pub_goal.publish(self._make_pose(wp['x'], wp['y'], wp.get('yaw', 0.0)))

    def _send_dock(self, dock_name):
        dp = self._docks.get(dock_name)
        if not dp:
            return
        self.get_logger().info(f'Sending dock goal → {dock_name}  ({dp["x"]}, {dp["y"]})')
        self._pub_state.publish(self._str(f'DOCKING:{dock_name}'))
        self._pub_goal.publish(self._make_pose(dp['x'], dp['y'], dp.get('yaw', 0.0)))

    def _make_pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(float(yaw))
        p.pose.orientation.x = qx
        p.pose.orientation.y = qy
        p.pose.orientation.z = qz
        p.pose.orientation.w = qw
        return p

    def _str(self, s):
        m = String(); m.data = s; return m


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
