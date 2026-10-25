#!/usr/bin/env python3
"""
map_autosaver.py

Continuously caches the latest /map OccupancyGrid in memory and writes the
standard Nav2 .pgm + .yaml files + updates maps/latest_map.txt:
  1. Whenever the /save_map_now (std_srvs/Empty) service is called (by explorer), OR
  2. Automatically when the node receives SIGINT / SIGTERM (Ctrl+C exit).

Because it caches the latest OccupancyGrid in RAM, saving on Ctrl+C is
instantaneous and never fails even if slam_toolbox shuts down first.
"""

import math
import os
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Empty


class MapAutosaver(Node):
    def __init__(self):
        super().__init__('map_autosaver')

        self.declare_parameter('map_save_prefix', '/tmp/agrobot_map')
        self.declare_parameter('maps_dir', '/tmp')
        self.declare_parameter('map_name', 'agrobot_map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('free_thresh', 0.25)
        self.declare_parameter('occupied_thresh', 0.65)

        self.map_save_prefix = self.get_parameter('map_save_prefix').value
        self.maps_dir = self.get_parameter('maps_dir').value
        self.map_name = self.get_parameter('map_name').value
        self.map_topic = self.get_parameter('map_topic').value
        self.free_thresh = float(self.get_parameter('free_thresh').value)
        self.occupied_thresh = float(self.get_parameter('occupied_thresh').value)

        self.latest_map: OccupancyGrid = None
        self.saved_once = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self.map_cb, qos
        )
        self.srv = self.create_service(Empty, '/save_map_now', self.save_srv_cb)

        self.get_logger().info(
            f"MapAutosaver active. Listening to {self.map_topic}. "
            f"Will auto-save to: {self.map_save_prefix}.[pgm|yaml]"
        )

    def map_cb(self, msg: OccupancyGrid):
        self.latest_map = msg

    def save_srv_cb(self, request, response):
        self.write_map_to_disk()
        return response

    def write_map_to_disk(self) -> bool:
        if self.latest_map is None:
            self.get_logger().error('Cannot save map: no /map message received yet!')
            return False

        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        q = msg.info.origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        os.makedirs(self.maps_dir, exist_ok=True)
        pgm_path = f"{self.map_save_prefix}.pgm"
        yaml_path = f"{self.map_save_prefix}.yaml"
        pgm_filename = os.path.basename(pgm_path)

        # Write binary PGM (P5) image: top row first (y from height-1 down to 0)
        pixels = bytearray(width * height)
        data = msg.data
        free_cell_max = int(self.free_thresh * 100.0)
        occ_cell_min = int(self.occupied_thresh * 100.0)

        idx = 0
        for y in range(height - 1, -1, -1):
            row_offset = y * width
            for x in range(width):
                val = data[row_offset + x]
                if val < 0:
                    pixels[idx] = 205  # Unknown (standard Nav2 gray)
                elif val >= occ_cell_min:
                    pixels[idx] = 0    # Occupied (black)
                elif val <= free_cell_max:
                    pixels[idx] = 254  # Free (white)
                else:
                    pixels[idx] = 205
                idx += 1

        with open(pgm_path, 'wb') as f:
            header = f"P5\n{width} {height}\n255\n".encode('ascii')
            f.write(header)
            f.write(pixels)

        # Write companion YAML file
        yaml_content = (
            f"image: {pgm_filename}\n"
            f"mode: trinary\n"
            f"resolution: {res:.6f}\n"
            f"origin: [{origin_x:.6f}, {origin_y:.6f}, {yaw:.6f}]\n"
            f"negate: 0\n"
            f"occupied_thresh: {self.occupied_thresh}\n"
            f"free_thresh: {self.free_thresh}\n"
        )
        with open(yaml_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)

        # Update latest_map.txt so localization mode picks it up automatically
        pointer_path = os.path.join(self.maps_dir, 'latest_map.txt')
        with open(pointer_path, 'w', encoding='utf-8') as f:
            f.write(self.map_name + '\n')

        self.saved_once = True
        self.get_logger().info(
            f"SUCCESS: Saved map ({width}x{height}) to {yaml_path} and updated latest_map.txt"
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MapAutosaver()

    def sig_handler(signum, frame):
        node.get_logger().info('Exit signal caught — saving map before shutdown...')
        node.write_map_to_disk()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if not node.saved_once:
            node.write_map_to_disk()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
