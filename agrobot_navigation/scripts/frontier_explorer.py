#!/usr/bin/env python3
"""
frontier_explorer.py

Autonomous Frontier Exploration node for ROS 2 Jazzy + Nav2:
  1. Subscribes to /map (OccupancyGrid) and looks up map -> base_footprint TF.
  2. Finds frontier clusters (free cells bordering unknown space) with clearance
     from lethal obstacles.
  3. Sends NavigateToPose goals to Nav2 and blacklists unreachable frontiers.
  4. Once no reachable frontiers remain, calls /save_map_now and exits with 0.
"""

from collections import deque
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('min_frontier_size', 8)        # cells
        self.declare_parameter('obstacle_clearance_m', 0.10)  # meters away from walls
        self.declare_parameter('goal_timeout_s', 45.0)
        self.declare_parameter('blacklist_radius_m', 0.20)
        self.declare_parameter('max_empty_checks', 3)         # consecutive checks before done

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_cluster = int(self.get_parameter('min_frontier_size').value)
        self.clearance_m = float(self.get_parameter('obstacle_clearance_m').value)
        self.goal_timeout = float(self.get_parameter('goal_timeout_s').value)
        self.blacklist_radius = float(self.get_parameter('blacklist_radius_m').value)
        self.max_empty_checks = int(self.get_parameter('max_empty_checks').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map_msg: OccupancyGrid = None
        self.blacklist = []  # list of (x, y) failed goals
        self.nav_in_progress = False
        self.current_goal_xy = None
        self.goal_handle = None
        self.goal_sent_time_ns = 0
        self.empty_frontier_count = 0
        self.finished = False

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_cb, 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.save_client = self.create_client(Empty, '/save_map_now')

        self.timer = self.create_timer(2.0, self.explore_tick)
        self.get_logger().info('Frontier Explorer started. Waiting for /map and Nav2...')

    def map_cb(self, msg: OccupancyGrid):
        self.map_msg = msg

    def get_robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time()
            )
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

    def is_blacklisted(self, wx: float, wy: float) -> bool:
        for bx, by in self.blacklist:
            if math.hypot(wx - bx, wy - by) < self.blacklist_radius:
                return True
        return False

    def find_frontiers(self, msg: OccupancyGrid):
        """Return list of (centroid_x, centroid_y, cluster_size) in world coordinates."""
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        data = msg.data

        clearance_cells = max(1, int(self.clearance_m / res))

        def is_safe_free(cx: int, cy: int) -> bool:
            if data[cy * w + cx] != 0:
                return False
            # Check obstacle clearance box
            for dy in range(-clearance_cells, clearance_cells + 1):
                ny = cy + dy
                if ny < 0 or ny >= h:
                    continue
                row = ny * w
                for dx in range(-clearance_cells, clearance_cells + 1):
                    nx = cx + dx
                    if 0 <= nx < w and data[row + nx] > 50:
                        return False
            return True

        def touches_unknown(cx: int, cy: int) -> bool:
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h and data[ny * w + nx] == -1:
                    return True
            return False

        visited = bytearray(w * h)
        clusters = []

        for y in range(1, h - 1):
            for x in range(1, w - 1):
                idx = y * w + x
                if visited[idx] or data[idx] != 0:
                    continue
                if not touches_unknown(x, y) or not is_safe_free(x, y):
                    continue

                # BFS cluster expansion
                q = deque([(x, y)])
                visited[idx] = 1
                cluster = []

                while q:
                    cx, cy = q.popleft()
                    cluster.append((cx, cy))
                    for dx in (-1, 0, 1):
                        for dy in (-1, 0, 1):
                            if dx == 0 and dy == 0:
                                continue
                            nx, ny = cx + dx, cy + dy
                            if 1 <= nx < w - 1 and 1 <= ny < h - 1:
                                nidx = ny * w + nx
                                if not visited[nidx] and data[nidx] == 0:
                                    if touches_unknown(nx, ny) and is_safe_free(nx, ny):
                                        visited[nidx] = 1
                                        q.append((nx, ny))

                if len(cluster) >= self.min_cluster:
                    # Pick the median cell in the cluster so the goal stays on valid free space
                    cluster.sort()
                    mx, my = cluster[len(cluster) // 2]
                    wx = ox + (mx + 0.5) * res
                    wy = oy + (my + 0.5) * res
                    if not self.is_blacklisted(wx, wy):
                        clusters.append((wx, wy, len(cluster)))

        return clusters

    def explore_tick(self):
        if self.finished or self.map_msg is None:
            return

        if not self.nav_client.server_is_ready():
            self.get_logger().info(
                'Waiting for NavigateToPose action server...', throttle_duration_sec=5.0
            )
            return

        robot_xy = self.get_robot_xy()
        if robot_xy is None:
            self.get_logger().info(
                f'Waiting for TF {self.map_frame} -> {self.base_frame}...',
                throttle_duration_sec=5.0,
            )
            return

        # Monitor active goal timeout
        if self.nav_in_progress:
            elapsed = (self.get_clock().now().nanoseconds - self.goal_sent_time_ns) / 1e9
            if elapsed > self.goal_timeout:
                self.get_logger().warn('Frontier goal timed out; blacklisting and replanning.')
                if self.current_goal_xy:
                    self.blacklist.append(self.current_goal_xy)
                if self.goal_handle is not None:
                    self.goal_handle.cancel_goal_async()
                self.nav_in_progress = False
            return

        frontiers = self.find_frontiers(self.map_msg)
        if not frontiers:
            self.empty_frontier_count += 1
            self.get_logger().info(
                f'No reachable frontiers found ({self.empty_frontier_count}/{self.max_empty_checks})'
            )
            if self.empty_frontier_count >= self.max_empty_checks:
                self.finish_exploration()
            return

        self.empty_frontier_count = 0

        # Score frontiers: prefer closer frontiers with a bonus for larger size
        rx, ry = robot_xy
        frontiers.sort(key=lambda f: math.hypot(f[0] - rx, f[1] - ry) - 0.05 * f[2])
        target_x, target_y, size = frontiers[0]
        self.send_goal(target_x, target_y, rx, ry, size)

    def send_goal(self, gx: float, gy: float, rx: float, ry: float, size: int):
        yaw = math.atan2(gy - ry, gx - rx)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)

        self.current_goal_xy = (gx, gy)
        self.nav_in_progress = True
        self.goal_sent_time_ns = self.get_clock().now().nanoseconds

        self.get_logger().info(
            f'Exploring frontier at ({gx:.2f}, {gy:.2f}) [size={size} cells, '
            f'remaining clusters={len(self.blacklist)} blacklisted]'
        )
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2; blacklisting frontier.')
            if self.current_goal_xy:
                self.blacklist.append(self.current_goal_xy)
            self.nav_in_progress = False
            return

        self.goal_handle = goal_handle
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self.goal_result_cb)

    def goal_result_cb(self, future):
        status = future.result().status
        # STATUS_SUCCEEDED == 4
        if status != 4 and self.current_goal_xy:
            self.get_logger().warn(
                f'Goal ended with status {status}; blacklisting ({self.current_goal_xy[0]:.2f}, '
                f'{self.current_goal_xy[1]:.2f})'
            )
            self.blacklist.append(self.current_goal_xy)
        else:
            self.get_logger().info('Reached frontier!')
        self.nav_in_progress = False

    def finish_exploration(self):
        self.get_logger().info('=== Exploration complete! Saving map via /save_map_now ===')
        self.timer.cancel()
        if self.save_client.wait_for_service(timeout_sec=5.0):
            fut = self.save_client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
