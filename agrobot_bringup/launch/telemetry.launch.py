#!/usr/bin/env python3
"""
Telemetry / operator-side launch for the SECOND RPi (the one that is not wired to the ESP32).

  ros2 launch agrobot_bringup telemetry.launch.py
  ros2 launch agrobot_bringup telemetry.launch.py autonomy:=true                 # explore (default)
  ros2 launch agrobot_bringup telemetry.launch.py autonomy:=true mode:=localization map_name:=field1
  mode:=explore | mapping | localization | gps  (explore = autonomy mapping + explore:=true)

DDS discovery is limited to the RPis, so the laptop never joins the ROS graph: it only opens a
Foxglove Studio websocket to  ws://<this-rpi>:8765 . Everything the Arm Planner panel needs is
therefore started here:

  lidar_manager (+ arm filter)  owns sllidar_node: /lidar_manager/{start,stop,restart,status,log}, so the
                                lidar can be stopped / relaunched and its logs read from Foxglove;
                                /scan_raw -> /scan
  foxglove_bridge               websocket 8765, topics / TF / services / client publishing
  arm_commander                 /arm_commander/{plan,execute,stop,status}, wraps MoveIt's
                                /plan_kinematic_path and /execute_trajectory as plain services
  telemetry_monitor             /diagnostics and /telemetry/* (CPU/temperature shown are of THIS RPi)

Stays on the hardware RPi (hardware.launch.py, moveit:=true): move_group (/compute_ik,
/plan_kinematic_path, /execute_trajectory, /robot_description_semantic), robot_state_publisher
(/robot_description) and the controllers. This launch reaches them over DDS.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration


def _include(pkg, name, args=None, condition=None):
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(args or {}).items(), condition=condition)


def _autonomy(context):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    if mode not in ('explore', 'mapping', 'localization', 'gps'):
        raise RuntimeError(f"mode:='{mode}' invalid; use explore, mapping, localization or gps")
    if LaunchConfiguration('autonomy').perform(context).strip().lower() not in ('true', '1', 'yes'):
        return []
    return [_include('agrobot_navigation', 'autonomy.launch.py', {
        'mode': 'mapping' if mode == 'explore' else mode,
        'explore': 'true' if mode == 'explore' else 'false',
        'map_name': LaunchConfiguration('map_name'),
    })]


def generate_launch_description():
    lc = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true', description='Launch the RPLidar (+ arm filter)'),
        DeclareLaunchArgument('lidar_auto_restart', default_value='true',
                              description='Restart sllidar_node by itself after a crash (max 5 in a row, '
                                          'then wait for the Foxglove button)'),
        DeclareLaunchArgument('filter_arm', default_value='true',
                              description='Mask the camera arm out of the scan (/scan_raw -> /scan)'),
        DeclareLaunchArgument('port', default_value='8765', description='Foxglove websocket port'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Foxglove bridge listen address (reachable from the laptop)'),
        DeclareLaunchArgument('arm_commander', default_value='true',
                              description='Run the plan/execute services used by the Arm Planner panel'),
        DeclareLaunchArgument('monitor', default_value='true', description='Run telemetry_monitor'),
        DeclareLaunchArgument('autonomy', default_value='false',
                              description='Also launch agrobot_navigation autonomy.launch.py'),
        DeclareLaunchArgument('mode', default_value='explore',
                              description='With autonomy:=true: explore | mapping | localization | gps'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='Map name for mapping (save) / localization (load); empty = auto'),

        Node(package='agrobot_bringup', executable='lidar_manager.py', name='lidar_manager', output='log',
             parameters=[{'scan_topic': 'scan_raw',
                          'autostart': True,
                          'auto_restart': ParameterValue(lc('lidar_auto_restart'), value_type=bool)}],
             condition=IfCondition(lc('lidar'))),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain',
             name='scan_to_scan_filter_chain', output='log',
             parameters=[os.path.join(get_package_share_directory('agrobot_bringup'), 'config',
                                      'laser_filter.yaml')],
             remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
             condition=IfCondition(lc('filter_arm'))),
        _include('agrobot_telemetry', 'foxglove.launch.py', {
            'port': lc('port'), 'address': lc('address'),
            'arm_commander': lc('arm_commander'), 'monitor': lc('monitor')}),
        OpaqueFunction(function=_autonomy),
    ])
