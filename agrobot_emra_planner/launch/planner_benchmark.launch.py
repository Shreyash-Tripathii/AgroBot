"""Headless planner benchmark: map_server + planner_server (EMRA/NavFn/Smac2D/ThetaStar) + static TF.

  ros2 launch agrobot_emra_planner planner_benchmark.launch.py map:=/path/to/map.yaml
then, in another terminal:
  ros2 run agrobot_emra_planner benchmark_planners.py --map /path/to/map.yaml --pairs 100
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('agrobot_emra_planner')
    default_map = os.path.join(
        get_package_share_directory('agrobot_navigation'), 'maps', 'sim', 'sim_field.yaml')
    params = os.path.join(share, 'config', 'benchmark_params.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint']),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map')}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_bench',
             output='screen',
             parameters=[{'autostart': True, 'node_names': ['map_server', 'planner_server']}]),
    ])
