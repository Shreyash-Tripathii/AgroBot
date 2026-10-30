import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('agrobot_navigation')
    default_config_path = os.path.join(pkg_navigation, 'config', 'octomap.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    frame_id = LaunchConfiguration('frame_id')
    base_frame_id = LaunchConfiguration('base_frame_id')
    resolution = LaunchConfiguration('resolution')
    params_file = LaunchConfiguration('params_file')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Fixed world reference frame (e.g., map or odom)'
    )

    declare_base_frame_id_cmd = DeclareLaunchArgument(
        'base_frame_id',
        default_value='base_footprint',
        description='Robot root base footprint frame'
    )

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value='0.05',
        description='Octree voxel resolution in meters (0.05 = 5cm)'
    )

    declare_pointcloud_topic_cmd = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/fusion_cloud/points',
        description='Input PointCloud2 topic name'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Full path to parameter YAML file'
    )

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server_node',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'frame_id': frame_id,
                'base_frame_id': base_frame_id,
                'resolution': resolution,
            }
        ],
        remappings=[
            ('cloud_in', pointcloud_topic),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(declare_base_frame_id_cmd)
    ld.add_action(declare_resolution_cmd)
    ld.add_action(declare_pointcloud_topic_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(octomap_server_node)

    return ld
