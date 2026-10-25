import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('agrobot_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('explore_params_file',
                              default_value=os.path.join(pkg, 'config', 'explore_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='explore_lite', executable='explore', name='explore_node',
             output='screen',
             parameters=[LaunchConfiguration('explore_params_file'),
                         {'use_sim_time': ParameterValue(
                             LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
