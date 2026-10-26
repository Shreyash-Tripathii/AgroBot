#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from geographic_msgs.msg import GeoPoint
from nav2_msgs.action import NavigateToPose
from robot_localization.srv import FromLL


class ClickGoal(Node):
    def __init__(self):
        super().__init__("click_goal")
        self.cli = self.create_client(FromLL, "/fromLL")
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(PointStamped, "/clicked_point", self.on_click, 10)
        self.get_logger().info("Waiting for clicks on /clicked_point (x=lon, y=lat)")

    def on_click(self, msg):
        if not self.cli.service_is_ready():
            self.get_logger().warn("/fromLL not available (is navsat_transform_node running?)")
            return
        req = FromLL.Request()
        req.ll_point = GeoPoint(latitude=msg.point.y, longitude=msg.point.x, altitude=0.0)
        self.cli.call_async(req).add_done_callback(self.on_converted)

    def on_converted(self, fut):
        p = fut.result().map_point
        self.get_logger().info(f"Goal in map frame: x={p.x:.2f} y={p.y:.2f}")
        if not self.nav.server_is_ready():
            self.get_logger().warn("navigate_to_pose action server not ready")
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = p.x
        goal.pose.pose.position.y = p.y
        goal.pose.pose.orientation.w = 1.0
        self.nav.send_goal_async(goal)


def main():
    rclpy.init()
    rclpy.spin(ClickGoal())

if __name__ == "__main__":
    main()
