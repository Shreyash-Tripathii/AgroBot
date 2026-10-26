#!/usr/bin/env python3
"""
One-shot GPS visualisation bringup (run on the laptop):
  1. Starts the danielsnider/mapproxy Docker container, unless it is already running
  2. Starts mapviz (loading a saved config), initialize_origin and the map->origin TF
  3. Starts click_goal.py (mapviz click -> /fromLL -> Nav2 NavigateToPose)

Usage:
  ros2 launch agrobot_bringup gps_viz.launch.py
  ros2 launch agrobot_bringup gps_viz.launch.py start_mapproxy:=false
  ros2 launch agrobot_bringup gps_viz.launch.py mapviz_config:=/path/to/other.mvc
"""
import os
import shutil
import subprocess
import time
import urllib.request

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TAG = "[gps_viz]"
TILE_TEST_URL = "http://localhost:8080/wmts/gm_layer/gm_grid/0/0/0.png"


def _docker(*args, timeout=30):
    return subprocess.run(["docker", *args], capture_output=True, text=True, timeout=timeout)


def _tiles_ready(timeout):
    end = time.time() + timeout
    while True:
        try:
            with urllib.request.urlopen(TILE_TEST_URL, timeout=2) as r:
                if r.status == 200:
                    return True
        except Exception:
            pass
        if time.time() >= end:
            return False
        time.sleep(1.0)


def ensure_mapproxy(container, image, cache_dir):
    """Make sure the map proxy is running. Never raises; returns LogInfo actions."""
    logs = []
    if shutil.which("docker") is None:
        return [LogInfo(msg=f"{TAG} WARNING: docker not installed, skipping map proxy")]

    try:
        # Any running container from this image counts (also catches one you
        # started by hand earlier, whatever its name).
        by_image = _docker("ps", "-q", "--filter", f"ancestor={image}")
        if by_image.returncode != 0:
            return [LogInfo(msg=f"{TAG} WARNING: docker not usable "
                                f"({by_image.stderr.strip()}). Is your user in the 'docker' group?")]
        by_name = _docker("ps", "-q", "--filter", f"name=^{container}$")

        if by_image.stdout.split() or by_name.stdout.split():
            logs.append(LogInfo(msg=f"{TAG} map proxy already running, not relaunching"))
            wait = 2.0
        else:
            stopped = _docker("ps", "-aq", "--filter", f"name=^{container}$")
            if stopped.stdout.split():
                r = _docker("start", container)
                action = f"restarted existing container '{container}'"
            else:
                os.makedirs(cache_dir, exist_ok=True)
                # long timeout: the first run has to pull the image
                r = _docker("run", "-d", "-t", "--name", container,
                            "-p", "8080:8080", "-v", f"{cache_dir}:/mapproxy",
                            image, timeout=300)
                action = f"started new container '{container}'"
            if r.returncode != 0:
                return [LogInfo(msg=f"{TAG} WARNING: could not start map proxy: {r.stderr.strip()}")]
            logs.append(LogInfo(msg=f"{TAG} {action}"))
            wait = 15.0
    except subprocess.TimeoutExpired:
        return [LogInfo(msg=f"{TAG} WARNING: docker command timed out, skipping map proxy")]

    if _tiles_ready(wait):
        logs.append(LogInfo(msg=f"{TAG} tile server is answering on :8080"))
    else:
        logs.append(LogInfo(msg=f"{TAG} WARNING: tile server not answering yet "
                                "(cold cache needs internet); mapviz will retry on its own"))
    return logs


def launch_setup(context, *args, **kwargs):
    cfg = lambda name: LaunchConfiguration(name).perform(context)
    actions = []

    if cfg("start_mapproxy").lower() == "true":
        actions += ensure_mapproxy(
            container=cfg("mapproxy_container"),
            image="danielsnider/mapproxy",
            cache_dir=os.path.expanduser(cfg("mapproxy_cache_dir")),
        )

    mapviz_cfg = os.path.expanduser(cfg("mapviz_config"))
    mapviz_params = []
    if os.path.isfile(mapviz_cfg):
        mapviz_params = [{"config": mapviz_cfg}]
        actions.append(LogInfo(msg=f"{TAG} mapviz config: {mapviz_cfg}"))
    else:
        actions.append(LogInfo(msg=f"{TAG} WARNING: {mapviz_cfg} not found, "
                                   "mapviz starts empty (save your layout with File > Save Config As)"))

    actions += [
        Node(package="mapviz", executable="mapviz", name="mapviz",
             output="screen", parameters=mapviz_params),
        Node(package="swri_transform_util", executable="initialize_origin.py",
             name="initialize_origin",
             remappings=[("fix", cfg("gps_topic"))]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             name="swri_transform",
             arguments=["--frame-id", "map", "--child-frame-id", "origin"]),
        Node(package="agrobot_bringup", executable="click_goal.py",
             name="click_goal", output="screen"),
    ]
    return actions


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory("agrobot_bringup"), "config", "mapviz.mvc")
    return LaunchDescription([
        DeclareLaunchArgument("start_mapproxy", default_value="true",
                              description="Start the Docker map proxy if it isn't running"),
        DeclareLaunchArgument("mapproxy_container", default_value="mapproxy"),
        DeclareLaunchArgument("mapproxy_cache_dir", default_value="~/mapproxy"),
        DeclareLaunchArgument("mapviz_config", default_value=default_cfg),
        DeclareLaunchArgument("gps_topic", default_value="/gps/fix"),
        OpaqueFunction(function=launch_setup),
    ])
