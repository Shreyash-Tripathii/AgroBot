#!/usr/bin/env python3
"""
Auto-detect the RPLidar A1M8 among /dev/ttyUSB* ports (by protocol handshake, not by
USB port path) and launch sllidar_node on it. Lidar only: the ESP32 port is handled by
hardware.launch.py on the robot RPi, so this file has no ESP32 logic.

How: for every /dev/ttyUSB*, open at 115200, send RPLidar STOP + GET_INFO and
look for the RPLidar response descriptor (A5 5A 14 00 00 00 04). Only a real
RPLidar answers that. Every other port is treated as "not the lidar".

Requires: python3-serial (sudo apt install python3-serial), user in `dialout`,
          sllidar_ros2 (Slamtec, built in your workspace)
Usage:    ros2 launch <your_pkg> rplidar_autodetect.launch.py
"""
import glob
import os
import time

import serial
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

STOP_CMD = b"\xA5\x25"
GET_INFO_CMD = b"\xA5\x50"
INFO_DESCRIPTOR = b"\xA5\x5A\x14\x00\x00\x00\x04"


def probe_rplidar(port: str, baud: int, window: float = 0.8) -> bool:
    """Return True if `port` answers the RPLidar GET_INFO handshake."""
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baud
    ser.timeout = 0.1
    ser.write_timeout = 0.5
    ser.exclusive = True       # skip ports already held by another process
    ser.dtr = False            # don't toggle auto-reset lines on the ESP32
    ser.rts = False
    try:
        ser.open()
        ser.reset_input_buffer()
        ser.write(STOP_CMD)    # in case the lidar is already scanning
        time.sleep(0.05)
        ser.reset_input_buffer()
        ser.write(GET_INFO_CMD)

        buf = b""
        deadline = time.monotonic() + window
        while time.monotonic() < deadline:
            buf += ser.read(64)
            if INFO_DESCRIPTOR in buf:
                return True
            if len(buf) > 2048:   # chatty device (e.g. ESP32 logs) -> not a lidar
                buf = buf[-64:]
        return False
    except (serial.SerialException, OSError):
        return False
    finally:
        if ser.is_open:
            ser.close()


def find_rplidar(baud: int, timeout: float):
    """Poll until the lidar is found or `timeout` seconds pass.
    Returns (lidar_port | None, [other_ports])."""
    deadline = time.monotonic() + timeout
    others = []
    while True:
        ports = sorted(glob.glob("/dev/ttyUSB*"))
        others = []
        for p in ports:
            if probe_rplidar(p, baud):
                return p, [q for q in ports if q != p]
            others.append(p)
        if time.monotonic() >= deadline:
            return None, others
        time.sleep(0.5)    # devices may still be enumerating at boot


def launch_setup(context, *args, **kwargs):
    baud = int(LaunchConfiguration("serial_baudrate").perform(context))
    timeout = float(LaunchConfiguration("detect_timeout").perform(context))

    lidar_port, others = find_rplidar(baud, timeout)

    # No lidar found: don't error out, just skip the lidar node.
    if lidar_port is None:
        return [
            LogInfo(msg=(
                f"[rplidar_autodetect] WARNING: RPLidar not found on any /dev/ttyUSB* "
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
