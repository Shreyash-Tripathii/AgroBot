#!/usr/bin/env python3
"""
RPLidar A1M8 launch (sllidar_ros2) + arm filter.

The lidar port is the /dev/ttyUSB* whose USB vendor is Silicon Labs (10c4, the CP210x bridge of the
RPLidar adapter board). Nothing is opened or probed: only the USB descriptors in sysfs are read, so no
other device (e.g. the ESP32 on the other RPi) is touched and the lidar sees no extra traffic.

Usage:    ros2 launch agrobot_bringup laser.launch.py
Requires: sllidar_ros2 (Slamtec, built in your workspace), laser_filters
"""
import glob
import os
import time

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CP210X_VENDOR = "10c4"


def usb_vendor(tty: str):
    """USB idVendor of /dev/<tty> from sysfs (walks up from the tty device to the USB device)."""
    path = os.path.realpath(f"/sys/class/tty/{tty}/device")
    while path and path != "/":
        f = os.path.join(path, "idVendor")
        if os.path.isfile(f):
            with open(f) as fh:
                return fh.read().strip().lower()
        path = os.path.dirname(path)
    return None


def find_rplidar(baud: int, timeout: float):
    """Poll until a CP210x /dev/ttyUSB* shows up or `timeout` seconds pass.
    Returns (port | None, [other ttyUSB ports])."""
    deadline = time.monotonic() + timeout
    while True:
        ports = sorted(glob.glob("/dev/ttyUSB*"))
        for p in ports:
            if usb_vendor(os.path.basename(p)) == CP210X_VENDOR:
                return p, [q for q in ports if q != p]
        if time.monotonic() >= deadline:
            return None, ports
        time.sleep(0.5)    # devices may still be enumerating at boot


def launch_setup(context, *args, **kwargs):
    baud = int(LaunchConfiguration("serial_baudrate").perform(context))
    timeout = float(LaunchConfiguration("detect_timeout").perform(context))

    lidar_port, others = find_rplidar(baud, timeout)

    # No lidar found: don't error out, just skip the lidar node.
    if lidar_port is None:
        return [
            LogInfo(msg=(
                f"[rplidar_autodetect] WARNING: No CP210x (Silicon Labs) /dev/ttyUSB* found "
                f"after {timeout:.0f}s (checked: {others or 'no ttyUSB devices'}). "
                "Not launching sllidar_node. Check wiring/power, `dialout` group "
                "membership, and that no other process holds the port.")),
        ]

    # filter_arm: lidar publishes /scan_raw; a box filter drops the arm and republishes /scan,
    # so Nav2 / SLAM / EKF configs keep using /scan unchanged.
    filter_arm = LaunchConfiguration("filter_arm").perform(context).lower() == "true"

    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        output="screen",
        parameters=[{
            "channel_type": "serial",
            "serial_port": lidar_port,
            "serial_baudrate": baud,
            "frame_id": LaunchConfiguration("frame_id"),
            "inverted": LaunchConfiguration("inverted"),
            "angle_compensate": True,
        }],
        remappings=[("scan", "scan_raw")] if filter_arm else [],
    )
    filter_nodes = []
    if filter_arm:
        filter_nodes.append(Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",  # must match the key in laser_filter.yaml
            output="screen",
            parameters=[os.path.join(
                get_package_share_directory("agrobot_bringup"), "config", "laser_filter.yaml")],
            remappings=[("scan", "scan_raw"), ("scan_filtered", "scan")],
        ))
    return [
        LogInfo(msg=f"[rplidar_autodetect] RPLidar found on {lidar_port}"),
        lidar_node,
        *filter_nodes,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("frame_id", default_value="lidar_link"),
        DeclareLaunchArgument("inverted", default_value="false"),
        DeclareLaunchArgument("detect_timeout", default_value="10.0",
                              description="Seconds to wait for the lidar to enumerate"),
        DeclareLaunchArgument("filter_arm", default_value="true",
                              description="Mask the camera arm out of the scan (/scan_raw -> /scan)"),
        OpaqueFunction(function=launch_setup),
    ])
