#!/usr/bin/env python3
"""
Auto-detect the RPLidar A1M8 among identical CP210x /dev/ttyUSB* ports
(by protocol handshake, not by USB port path) and launch sllidar_node on it.

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


def make_symlink(link: str, target: str, fallback: str):
    """Atomically (re)point `link` at `target`. /dev is root-only, so if that
    fails, fall back to a user-writable path. Returns (path_used, warning|None)."""
    for path in (link, fallback):
        tmp = f"{path}.tmp{os.getpid()}"
        try:
            if os.path.lexists(tmp):
                os.remove(tmp)
            os.symlink(target, tmp)
            os.replace(tmp, path)          # atomic swap, replaces stale links
            warn = None if path == link else (
                f"could not create {link} (needs root); using {path} instead")
            return path, warn
        except OSError:
            if os.path.lexists(tmp):
                os.remove(tmp)
    return None, f"could not create symlink at {link} or {fallback}"

def esp_link_logs(others, link, fallback):
    """Symlink the single non-lidar port to `link` (falls back to `fallback`).
    Returns a list of LogInfo actions."""
    if len(others) != 1:
        return [LogInfo(msg=(
            f"[rplidar_autodetect] WARNING: expected exactly 1 non-lidar ttyUSB port, "
            f"found {others}; not creating {link}"))]
    used, warn = make_symlink(link, others[0], fallback)
    logs = []
    if used:
        logs.append(LogInfo(msg=f"[rplidar_autodetect] {used} -> {others[0]}"))
    if warn:
        logs.append(LogInfo(msg=f"[rplidar_autodetect] WARNING: {warn}"))
    return logs


def launch_setup(context, *args, **kwargs):
    baud = int(LaunchConfiguration("serial_baudrate").perform(context))
    timeout = float(LaunchConfiguration("detect_timeout").perform(context))

    esp_link = LaunchConfiguration("esp32_symlink").perform(context)
    esp_fallback = LaunchConfiguration("esp32_symlink_fallback").perform(context)

    lidar_port, others = find_rplidar(baud, timeout)

    # No lidar found: don't error out. Treat the remaining port as the ESP32,
    # link it at /tmp/esp32 (the fallback path), and skip launching the lidar node.
    if lidar_port is None:
        return [
            LogInfo(msg=(
                f"[rplidar_autodetect] WARNING: RPLidar not found on any /dev/ttyUSB* "
                f"after {timeout:.0f}s (checked: {others or 'no ttyUSB devices'}). "
                "Not launching sllidar_node. Check wiring/power, `dialout` group "
                "membership, and that no other process holds the port.")),
            *esp_link_logs(others, esp_fallback, esp_fallback),
        ]

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
    )
    return [
        LogInfo(msg=f"[rplidar_autodetect] RPLidar found on {lidar_port}"),
        LogInfo(msg=f"[rplidar_autodetect] Remaining ttyUSB ports (ESP32 candidate): {others}"),
        *esp_link_logs(others, esp_link, esp_fallback),
        lidar_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("serial_baudrate", default_value="115200"),
        DeclareLaunchArgument("frame_id", default_value="lidar_link"),
        DeclareLaunchArgument("inverted", default_value="false"),
        DeclareLaunchArgument("detect_timeout", default_value="10.0",
                              description="Seconds to wait for the lidar to enumerate"),
        DeclareLaunchArgument("esp32_symlink", default_value="/dev/esp32",
                              description="Symlink to create for the non-lidar port"),
        DeclareLaunchArgument("esp32_symlink_fallback", default_value="/tmp/esp32",
                              description="Used if esp32_symlink can't be created (no root)"),
        OpaqueFunction(function=launch_setup),
    ])
