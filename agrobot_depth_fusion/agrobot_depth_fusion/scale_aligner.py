import logging
from typing import Tuple, Optional
import numpy as np


class ScaleAligner:
    """
    Robust Sparse-to-Dense Scale and Shift Aligner.
    Maps monocular relative depth (d_rel) to metric LiDAR depth (d_lidar):
        d_metric = s * d_rel + t
    """

    def __init__(
        self,
        method: str = "ransac",
        ransac_threshold: float = 0.15,
        ransac_iterations: int = 150,
        min_inliers: int = 5,
        smooth_factor: float = 0.85,
        min_depth: float = 0.2,
        max_depth: float = 12.0,
    ):
        self.method = method.lower()
        self.ransac_threshold = ransac_threshold
        self.ransac_iterations = ransac_iterations
        self.min_inliers = min_inliers
        self.smooth_factor = smooth_factor
        self.min_depth = min_depth
        self.max_depth = max_depth

        self.s_prev: Optional[float] = None
        self.t_prev: Optional[float] = None

        self.last_inlier_ratio: float = 0.0
        self.last_num_points: int = 0

    def fit(self, d_rel: np.ndarray, d_lidar: np.ndarray) -> Tuple[float, float]:
        """
        Calculates scale (s) and shift (t) mapping relative depth to metric depth.
        Args:
            d_rel: np.ndarray of sampled relative depths at LiDAR points
            d_lidar: np.ndarray of corresponding metric LiDAR depths (Z in camera frame)
        Returns:
            (s, t): scale factor and shift offset
        """
        # Filter valid, positive, finite pairs
        valid_mask = (
            np.isfinite(d_rel)
            & np.isfinite(d_lidar)
            & (d_rel > 1e-4)
            & (d_lidar >= self.min_depth)
            & (d_lidar <= self.max_depth)
        )
        d_rel_v = d_rel[valid_mask]
        d_lidar_v = d_lidar[valid_mask]
        n_pts = len(d_rel_v)
        self.last_num_points = n_pts

        if n_pts < 3:
            # Insufficient points: reuse previous estimate or fallback
            if self.s_prev is not None and self.t_prev is not None:
                return self.s_prev, self.t_prev
            return 1.0, 0.0

        s_est, t_est = None, None

        if self.method == "ransac" and n_pts >= self.min_inliers:
            s_est, t_est = self._fit_ransac(d_rel_v, d_lidar_v)

        # Fallback to robust median ratio if RANSAC fails or is unselected
        if s_est is None or t_est is None:
            s_est, t_est = self._fit_median_ratio(d_rel_v, d_lidar_v)

        # Apply exponential moving average smoothing
        if self.s_prev is not None and self.t_prev is not None and 0.0 < self.smooth_factor < 1.0:
            s_est = self.smooth_factor * s_est + (1.0 - self.smooth_factor) * self.s_prev
            t_est = self.smooth_factor * t_est + (1.0 - self.smooth_factor) * self.t_prev

        self.s_prev = s_est
        self.t_prev = t_est
        return s_est, t_est

    def _fit_ransac(self, d_rel: np.ndarray, d_lidar: np.ndarray) -> Tuple[Optional[float], Optional[float]]:
        n_pts = len(d_rel)
        best_inliers_count = 0
        best_inlier_mask = None

        for _ in range(self.ransac_iterations):
            idx = np.random.choice(n_pts, size=2, replace=False)
            x1, x2 = d_rel[idx[0]], d_rel[idx[1]]
            y1, y2 = d_lidar[idx[0]], d_lidar[idx[1]]

            dx = x2 - x1
            if abs(dx) < 1e-4:
                continue

            s_cand = (y2 - y1) / dx
            t_cand = y1 - s_cand * x1

            # Check residual error
            pred = s_cand * d_rel + t_cand
            residuals = np.abs(pred - d_lidar)
            inlier_mask = (residuals < self.ransac_threshold) & (pred > 0)
            inliers_count = np.count_nonzero(inlier_mask)

            if inliers_count > best_inliers_count:
                best_inliers_count = inliers_count
                best_inlier_mask = inlier_mask

        self.last_inlier_ratio = best_inliers_count / float(n_pts) if n_pts > 0 else 0.0

        if best_inliers_count >= self.min_inliers and best_inlier_mask is not None:
            # Refit on all inliers using least squares
            x_in = d_rel[best_inlier_mask]
            y_in = d_lidar[best_inlier_mask]
            coeffs = np.polyfit(x_in, y_in, deg=1)
            s_opt, t_opt = float(coeffs[0]), float(coeffs[1])
            return s_opt, t_opt

        return None, None

    def _fit_median_ratio(self, d_rel: np.ndarray, d_lidar: np.ndarray) -> Tuple[float, float]:
        ratios = d_lidar / (d_rel + 1e-6)
        s_median = float(np.median(ratios))
        t_shift = 0.0
        self.last_inlier_ratio = 1.0
        return s_median, t_shift

    def scale_depth(self, relative_depth: np.ndarray, s: float, t: float) -> np.ndarray:
        """
        Converts full relative depth map into dense metric depth map.
        """
        dense_metric = s * relative_depth + t
        np.clip(dense_metric, self.min_depth, self.max_depth, out=dense_metric)
        return dense_metric
