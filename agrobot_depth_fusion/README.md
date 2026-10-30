# agrobot_depth_fusion

Sparse-to-Dense Scale Alignment Pipeline fusing 2D LiDAR (`sensor_msgs/LaserScan`) and monocular camera (`sensor_msgs/Image`) into a metric colored 3D point cloud (`sensor_msgs/PointCloud2`).

## Overview

Monocular depth models like **Depth Anything V2** provide dense, high-frequency relative depth maps from standard 2D webcam images. However, monocular depth lacks metric scale and suffers from scale/shift drift.

This package:
1. Synchronizes `/image_raw` and `/scan` using `message_filters.ApproximateTimeSynchronizer`.
2. Infers dense relative depth using **Depth Anything V2 Small** (`vits` encoder) with PyTorch FP16 on CUDA (optimized for RTX 3050 4GB VRAM to hit 30+ FPS).
3. Projects 2D LiDAR scan points onto the camera image plane using camera intrinsics ($K$) and static extrinsics ($\Delta Z = 0.10$ m vertical offset).
4. Samples relative depth at the projected pixel coordinates $(u, v)$ with local patch filtering.
5. Performs robust scale and shift alignment ($d_{\text{metric}} = s \cdot d_{\text{rel}} + t$) via **RANSAC** (with fallback to robust median ratio) and temporal EMA smoothing.
6. Scaled metric depth map is back-projected using vectorized NumPy pinhole calculations into a colored 3D PointCloud array $[X, Y, Z, R, G, B]$.
7. Publishes `sensor_msgs/PointCloud2` to `/fusion_cloud/points`.

---

## Installation & Prerequisites

Inside your ROS 2 workspace (e.g. `~/agrobot_ws`):

```bash
# Install PyTorch with CUDA support (e.g. for CUDA 12.1/11.8)
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# Install OpenCV and sensor_msgs_py
sudo apt install python3-opencv python3-pip ros-${ROS_DISTRO}-sensor-msgs-py ros-${ROS_DISTRO}-cv-bridge
```

---

## Building the Package

```bash
cd ~/agrobot_ws
colcon build --packages-select agrobot_depth_fusion agrobot_description
source install/setup.bash
```

---

## Running the Node

Launch the fusion pipeline with default parameters:
```bash
ros2 launch agrobot_depth_fusion depth_fusion.launch.py
```

Launch with custom parameter overrides:
```bash
ros2 launch agrobot_depth_fusion depth_fusion.launch.py image_topic:=/camera/image_raw scan_topic:=/scan
```

View the generated point cloud in RViz2:
- Set Fixed Frame: `camera_optical_link` (or `base_footprint`)
- Add display: `PointCloud2`
- Topic: `/fusion_cloud/points`
- Color Transformer: `RGB8`
