#!/usr/bin/env python3
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import message_filters
from sensor_msgs_py import point_cloud2

from .depth_anything_v2 import load_depth_anything_v2
from .scale_aligner import ScaleAligner
from .projection_utils import (
    project_lidar_to_camera,
    sample_relative_depth,
    backproject_depth_to_colored_pointcloud,
)


class DepthFusionNode(Node):
    """
    Sparse-to-Dense Scale Alignment Node:
    Synchronizes 2D LiDAR /scan and webcam /image_raw.
    Estimates relative depth using Depth Anything V2 Small (FP16 on CUDA).
    Aligns scale and shift using projected LiDAR points via RANSAC.
    Generates and publishes scaled dense colored 3D PointCloud2 on /fusion_cloud/points.
    """

    def __init__(self):
        super().__init__('depth_fusion_node')

        # ---------------------------------------------------------
        # ROS 2 Parameters Declaration
        # ---------------------------------------------------------
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('pointcloud_topic', '/fusion_cloud/points')
        self.declare_parameter('camera_frame', 'camera_optical_link')

        # Camera Intrinsics K
        self.declare_parameter('camera.fx', 525.0)
        self.declare_parameter('camera.fy', 525.0)
        self.declare_parameter('camera.cx', 320.0)
        self.declare_parameter('camera.cy', 240.0)

        # Extrinsic Transform (LiDAR to Camera)
        # In agrobot, camera is placed directly above LiDAR at 10cm (+0.10m)
        self.declare_parameter('extrinsic.z_offset', 0.10)
        self.declare_parameter('extrinsic.x_offset', 0.0)
        self.declare_parameter('extrinsic.y_offset', 0.0)

        # Model & Inference Parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_encoder', 'vits')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_fp16', True)
        self.declare_parameter('input_size', 518)

        # Depth & PointCloud Filtering
        self.declare_parameter('min_depth', 0.2)
        self.declare_parameter('max_depth', 12.0)
        self.declare_parameter('pointcloud_stride', 1)

        # Alignment Parameters
        self.declare_parameter('alignment_method', 'ransac')
        self.declare_parameter('ransac_threshold', 0.15)
        self.declare_parameter('ransac_iterations', 150)
        self.declare_parameter('smooth_factor', 0.85)

        # Synchronization & Debug
        self.declare_parameter('sync_slop', 0.08)
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('publish_depth_image', True)
        self.declare_parameter('publish_debug_image', True)

        # ---------------------------------------------------------
        # Retrieve Parameters
        # ---------------------------------------------------------
        self.image_topic = self.get_parameter('image_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value

        fx = float(self.get_parameter('camera.fx').value)
        fy = float(self.get_parameter('camera.fy').value)
        cx = float(self.get_parameter('camera.cx').value)
        cy = float(self.get_parameter('camera.cy').value)
        self.K = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)

        self.extrinsic_z_offset = float(self.get_parameter('extrinsic.z_offset').value)
        self.extrinsic_x_offset = float(self.get_parameter('extrinsic.x_offset').value)
        self.extrinsic_y_offset = float(self.get_parameter('extrinsic.y_offset').value)

        model_path = self.get_parameter('model_path').value
        encoder = self.get_parameter('model_encoder').value
        device = self.get_parameter('device').value
        use_fp16 = bool(self.get_parameter('use_fp16').value)
        self.input_size = int(self.get_parameter('input_size').value)

        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.stride = int(self.get_parameter('pointcloud_stride').value)

        method = self.get_parameter('alignment_method').value
        ransac_thresh = float(self.get_parameter('ransac_threshold').value)
        ransac_iter = int(self.get_parameter('ransac_iterations').value)
        smooth = float(self.get_parameter('smooth_factor').value)

        self.publish_depth_img = bool(self.get_parameter('publish_depth_image').value)
        self.publish_debug_img = bool(self.get_parameter('publish_debug_image').value)
        sync_slop = float(self.get_parameter('sync_slop').value)
        sync_queue = int(self.get_parameter('sync_queue_size').value)

        self.bridge = CvBridge()

        # ---------------------------------------------------------
        # Initialize Depth Anything V2 Small Model
        # ---------------------------------------------------------
        self.get_logger().info(
            f"Loading Depth Anything V2 (encoder: '{encoder}', device: '{device}', FP16: {use_fp16})..."
        )
        self.model = load_depth_anything_v2(
            checkpoint_path=model_path,
            encoder=encoder,
            device=device,
            fp16=use_fp16
        )
        self.get_logger().info("Depth Anything V2 model loaded successfully.")

        # ---------------------------------------------------------
        # Initialize Scale Aligner
        # ---------------------------------------------------------
        self.aligner = ScaleAligner(
            method=method,
            ransac_threshold=ransac_thresh,
            ransac_iterations=ransac_iter,
            smooth_factor=smooth,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
        )

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------
        self.cloud_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, 10)

        if self.publish_depth_img:
            self.depth_img_pub = self.create_publisher(Image, '/fusion_cloud/depth', 10)
        if self.publish_debug_img:
            self.debug_img_pub = self.create_publisher(Image, '/fusion_cloud/debug_projection', 10)

        # ---------------------------------------------------------
        # Subscriptions & Approximate Time Synchronization
        # ---------------------------------------------------------
        self.image_sub = message_filters.Subscriber(self, Image, self.image_topic)
        self.scan_sub = message_filters.Subscriber(self, LaserScan, self.scan_topic)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.scan_sub],
            queue_size=sync_queue,
            slop=sync_slop
        )
        self.sync.registerCallback(self.sync_callback)

        self.point_fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        self.last_log_time = time.time()
        self.frame_count = 0
        self.get_logger().info(
            f"Depth Fusion Node initialized. Syncing [{self.image_topic}] and [{self.scan_topic}]. "
            f"Publishing PointCloud2 on [{self.pointcloud_topic}]."
        )

    def sync_callback(self, image_msg: Image, scan_msg: LaserScan):
        start_t = time.time()

        # 1. Convert Image Message
        try:
            cv_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            rgb_image = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        h, w = rgb_image.shape[:2]

        # 2. Monocular Depth Inference (Depth Anything V2 Small)
        t_infer_start = time.time()
        try:
            rel_depth_map = self.model.infer_image(rgb_image, input_size=self.input_size)
        except Exception as e:
            self.get_logger().error(f"Depth inference error: {e}")
            return
        t_infer = (time.time() - t_infer_start) * 1000.0

        # 3. Project 2D LiDAR Scan onto Camera Image Plane
        ranges = np.array(scan_msg.ranges, dtype=np.float32)
        u_px, v_px, metric_depths = project_lidar_to_camera(
            ranges=ranges,
            angle_min=scan_msg.angle_min,
            angle_increment=scan_msg.angle_increment,
            range_min=max(scan_msg.range_min, self.min_depth),
            range_max=min(scan_msg.range_max, self.max_depth),
            K=self.K,
            image_shape=(h, w),
            z_offset=self.extrinsic_z_offset,
            x_offset=self.extrinsic_x_offset,
            y_offset=self.extrinsic_y_offset,
        )

        # 4. Sample Relative Depth Map at Projected LiDAR Pixels
        if len(u_px) >= 3:
            sampled_rel_depth = sample_relative_depth(rel_depth_map, u_px, v_px, patch_size=3)
            # 5. Robust Metric Scale Alignment (RANSAC / Median Ratio)
            s, t = self.aligner.fit(sampled_rel_depth, metric_depths)
        else:
            s, t = self.aligner.fit(np.array([]), np.array([]))

        # 6. Apply Scale and Shift to Obtain Dense Metric Depth Map
        dense_metric_depth = self.aligner.scale_depth(rel_depth_map, s, t)

        # 7. Vectorized Back-Projection into Colored 3D Point Cloud [X, Y, Z, RGB]
        cloud_points = backproject_depth_to_colored_pointcloud(
            metric_depth=dense_metric_depth,
            rgb_image=rgb_image,
            K=self.K,
            stride=self.stride,
            min_depth=self.min_depth,
            max_depth=self.max_depth,
        )

        # 8. Construct and Publish sensor_msgs/PointCloud2
        header = Header()
        header.stamp = image_msg.header.stamp
        header.frame_id = self.camera_frame

        cloud_msg = point_cloud2.create_cloud(header, self.point_fields, cloud_points)
        self.cloud_pub.publish(cloud_msg)

        # Optional Depth Map & Debug Projection Publishing
        if self.publish_depth_img:
            depth_msg = self.bridge.cv2_to_imgmsg(dense_metric_depth.astype(np.float32), encoding='32FC1')
            depth_msg.header = header
            self.depth_img_pub.publish(depth_msg)

        if self.publish_debug_img and len(u_px) > 0:
            debug_bgr = cv_bgr.copy()
            for u, v, d in zip(u_px, v_px, metric_depths):
                cv2.circle(debug_bgr, (int(u), int(v)), 3, (0, 255, 0), -1)
            debug_msg = self.bridge.cv2_to_imgmsg(debug_bgr, encoding='bgr8')
            debug_msg.header = header
            self.debug_img_pub.publish(debug_msg)

        total_t = (time.time() - start_t) * 1000.0
        self.frame_count += 1
        if time.time() - self.last_log_time >= 2.0:
            fps = self.frame_count / (time.time() - self.last_log_time)
            self.get_logger().info(
                f"[Fusion] FPS: {fps:.1f} | Total: {total_t:.1f}ms (Infer: {t_infer:.1f}ms) | "
                f"LiDAR Pts: {len(u_px)} | Scale s={s:.4f}, Shift t={t:.4f}m | Points: {len(cloud_points)}"
            )
            self.frame_count = 0
            self.last_log_time = time.time()


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
