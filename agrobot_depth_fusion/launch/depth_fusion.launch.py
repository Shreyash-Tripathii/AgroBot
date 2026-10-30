import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_depth_fusion = get_package_share_directory('agrobot_depth_fusion')
    default_params_file = os.path.join(pkg_depth_fusion, 'config', 'depth_fusion_params.yaml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    image_topic = LaunchConfiguration('image_topic')
    scan_topic = LaunchConfiguration('scan_topic')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to parameter YAML file for depth_fusion_node'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_image_topic_cmd = DeclareLaunchArgument(
        'image_topic',
        default_value='/image_raw',
        description='Topic name for input RGB image'
    )

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic name for input 2D LiDAR LaserScan'
    )

    depth_fusion_node = Node(
        package='agrobot_depth_fusion',
        executable='depth_fusion_node',
        name='depth_fusion_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'image_topic': image_topic,
                'scan_topic': scan_topic,
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_image_topic_cmd)
    ld.add_action(declare_scan_topic_cmd)
    ld.add_action(depth_fusion_node)

    return ld
