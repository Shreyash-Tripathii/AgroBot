#!/usr/bin/env python3
"""
AgroBot telemetry for Foxglove Studio: bridge + monitor (+ open Studio).

  ros2 launch agrobot_telemetry foxglove.launch.py sim:=true studio:=true     # with the Gazebo sim
  ros2 launch agrobot_telemetry foxglove.launch.py studio:=true               # real robot (run on a PC
                                                                              #  or on the RPi)
In Foxglove Studio: Layouts -> Import from file... and pick
  <agrobot_telemetry>/foxglove/layouts/agrobot_mission_control.json   (dashboard)
  <agrobot_telemetry>/foxglove/layouts/agrobot_engineering.json       (plots / tuning)
or open Studio on ws://<host>:8765 from another PC when address/port are reachable.

Starts
  foxglove_bridge   ROS 2 <-> Foxglove websocket (topics, TF, parameters, services, /cmd_vel publish)
  arm_commander     /arm_commander/{plan,execute,stop,status}: MoveIt as services (Arm Planner panel)
  autonomy_manager  /autonomy_manager/*: start / stop autonomy.launch.py (explore, mapping, localization, gps)
  telemetry_monitor /diagnostics + /telemetry/* (system health, sensor rates, controller states)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    sim = ParameterValue(LaunchConfiguration('sim'), value_type=bool)
    port = ParameterValue(LaunchConfiguration('port'), value_type=int)

    bridge = Node(
        package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
        output='log',
        parameters=[{
            'port': port,
            'address': LaunchConfiguration('address'),
            'use_sim_time': sim,
            'send_buffer_limit': 200000000,   # large latched maps / images
            'max_qos_depth': 500,             # keep the lidar log history (/lidar_manager/log)
        }],
    )

    monitor = Node(
        package='agrobot_telemetry', executable='telemetry_monitor.py', name='telemetry_monitor',
        output='log',
        parameters=[{
            'use_sim_time': sim,
            'temp_warn_c': ParameterValue(LaunchConfiguration('temp_warn_c'), value_type=float),
            'temp_error_c': ParameterValue(LaunchConfiguration('temp_error_c'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('monitor')),
    )

    commander = Node(
        package='agrobot_telemetry', executable='arm_commander.py', name='arm_commander',
        output='log', parameters=[{'use_sim_time': sim}],
        condition=IfCondition(LaunchConfiguration('arm_commander')),
    )

    autonomy = Node(
        package='agrobot_telemetry', executable='autonomy_manager.py', name='autonomy_manager',
        output='screen', parameters=[{'use_sim_time': sim, 'sim': sim}],
        condition=IfCondition(LaunchConfiguration('autonomy_manager')),
    )

    studio = ExecuteProcess(
        cmd=['foxglove-studio',
             ['foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:', LaunchConfiguration('port')]],
        output='log', condition=IfCondition(LaunchConfiguration('studio')),
        additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
    )

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false', description='true with the Gazebo sim'),
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0',
                              description='Listen address (0.0.0.0 = reachable from other machines)'),
        DeclareLaunchArgument('monitor', default_value='true',
                              description='Run telemetry_monitor (/diagnostics, /telemetry/*)'),
        DeclareLaunchArgument('temp_warn_c', default_value='80.0',
                              description='CPU temperature that turns Health yellow'),
        DeclareLaunchArgument('temp_error_c', default_value='90.0',
                              description='CPU temperature that turns Health red'),
        DeclareLaunchArgument('arm_commander', default_value='true',
                              description='Run arm_commander (plan/execute services for the Arm Planner panel)'),
        DeclareLaunchArgument('autonomy_manager', default_value='true',
                              description='Run autonomy_manager (start / stop autonomy.launch.py from Foxglove)'),
        DeclareLaunchArgument('studio', default_value='false',
                              description='Open Foxglove Studio connected to the bridge'),
        bridge, monitor, commander, autonomy, studio,
    ])
