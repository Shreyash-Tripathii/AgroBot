from typing import Tuple
import numpy as np


def project_lidar_to_camera(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    K: np.ndarray,
    image_shape: Tuple[int, int],
    z_offset: float = 0.10,
    x_offset: float = 0.0,
    y_offset: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Projects 2D LiDAR scan beams onto the camera optical frame and image plane.
    Robot Body Frame: X forward, Y left, Z up.
    Camera Optical Frame: X right, Y down, Z forward.

    Extrinsic: Camera is located at z_offset (e.g. +10cm) above LiDAR.
    In camera optical coordinates:
        X_c = - (y_lidar - y_offset)
        Y_c = + z_offset (LiDAR plane is below camera, so positive Y in optical frame)
        Z_c = x_lidar - x_offset

    Returns:
        (u_px, v_px, metric_depths): pixel coordinates and corresponding Z depths (meters).
    """
    img_h, img_w = image_shape
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    # Generate beam angles
    indices = np.arange(len(ranges))
    angles = angle_min + indices * angle_increment
    valid_range = (ranges >= range_min) & (ranges <= range_max) & np.isfinite(ranges)

    r_valid = ranges[valid_range]
    theta_valid = angles[valid_range]

    # 2D LiDAR coordinates (in LiDAR sensor frame)
    x_l = r_valid * np.cos(theta_valid)
    y_l = r_valid * np.sin(theta_valid)

    # Transform into Camera Optical Frame
    X_c = - (y_l - y_offset)
    Y_c = np.full_like(X_c, fill_value=z_offset)
    Z_c = x_l - x_offset

    # Keep only points in front of camera
    front_mask = Z_c > 0.15
    X_c = X_c[front_mask]
    Y_c = Y_c[front_mask]
    Z_c = Z_c[front_mask]

    # Pinhole projection
    u = fx * (X_c / Z_c) + cx
    v = fy * (Y_c / Z_c) + cy

    # Filter within image boundaries
    in_bounds = (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)
    u_px = np.round(u[in_bounds]).astype(np.int32)
    v_px = np.round(v[in_bounds]).astype(np.int32)
    metric_depths = Z_c[in_bounds]

    return u_px, v_px, metric_depths


def sample_relative_depth(
    relative_depth_map: np.ndarray,
    u_px: np.ndarray,
    v_px: np.ndarray,
    patch_size: int = 3,
) -> np.ndarray:
    """
    Robustly samples relative depth map at projected pixels using a local median patch.
    """
    img_h, img_w = relative_depth_map.shape
    half = patch_size // 2
    sampled = np.zeros(len(u_px), dtype=np.float32)

    for i in range(len(u_px)):
        u = u_px[i]
        v = v_px[i]
        u_min = max(0, u - half)
        u_max = min(img_w, u + half + 1)
        v_min = max(0, v - half)
        v_max = min(img_h, v + half + 1)

        patch = relative_depth_map[v_min:v_max, u_min:u_max]
        sampled[i] = np.median(patch) if patch.size > 0 else relative_depth_map[v, u]

    return sampled


def backproject_depth_to_colored_pointcloud(
    metric_depth: np.ndarray,
    rgb_image: np.ndarray,
    K: np.ndarray,
    stride: int = 1,
    min_depth: float = 0.2,
    max_depth: float = 12.0,
) -> np.ndarray:
    """
    Vectorized NumPy back-projection of dense metric depth and RGB image to 3D point cloud.
    Returns:
        points_array: np.ndarray of shape (N, 4) with [X, Y, Z, packed_RGB_float32]
    """
    img_h, img_w = metric_depth.shape
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]

    # Coordinate grids with stride
    u_vals = np.arange(0, img_w, stride, dtype=np.float32)
    v_vals = np.arange(0, img_h, stride, dtype=np.float32)
    u_grid, v_grid = np.meshgrid(u_vals, v_vals)

    # Subsampled depth & valid mask
    Z = metric_depth[::stride, ::stride]
    valid_mask = (Z >= min_depth) & (Z <= max_depth) & np.isfinite(Z)

    u_pts = u_grid[valid_mask]
    v_pts = v_grid[valid_mask]
    Z_pts = Z[valid_mask]

    # Pinhole 3D back-projection
    X_pts = (u_pts - cx) * Z_pts / fx
    Y_pts = (v_pts - cy) * Z_pts / fy

    # Sample RGB colors
    colors = rgb_image[::stride, ::stride][valid_mask]
    r = colors[:, 0].astype(np.uint32)
    g = colors[:, 1].astype(np.uint32)
    b = colors[:, 2].astype(np.uint32)

    # Pack RGB into 32-bit float for sensor_msgs/PointCloud2
    packed_rgb = (r << 16) | (g << 8) | b
    rgb_float = packed_rgb.view(np.float32)

    points_cloud = np.column_stack([
        X_pts.astype(np.float32),
        Y_pts.astype(np.float32),
        Z_pts.astype(np.float32),
        rgb_float
    ])

    return points_cloud
