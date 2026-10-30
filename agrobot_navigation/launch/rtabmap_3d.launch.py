import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('agrobot_navigation')
    default_config_path = os.path.join(pkg_navigation, 'config', 'rtabmap_params.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    params_file = LaunchConfiguration('params_file')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_localization_cmd = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Run RTAB-Map in localization mode (true) or mapping mode (false)'
    )

    declare_pointcloud_topic_cmd = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/fusion_cloud/points',
        description='Input PointCloud2 topic name'
    )

    declare_odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='Odometry topic name (from EKF or base controller)'
    )

    declare_imu_topic_cmd = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/data',
        description='IMU topic name (e.g., from BNO085)'
    )

    declare_database_path_cmd = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap.db',
        description='Path to SQLite database file where 3D map is saved'
    )

    declare_delete_db_on_start_cmd = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='false',
        description='Delete existing map database on start (fresh mapping)'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_config_path,
        description='Path to rtabmap parameter YAML file'
    )

    declare_rtabmap_viz_cmd = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map 3D visualization GUI'
    )

    # 1. RTAB-Map Mapping Mode Node
    rtabmap_mapping_node = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'base_footprint',
                'database_path': database_path,
                'subscribe_imu': True,
                'Mem/IncrementalMemory': 'true',
                'Mem/InitWMWithAllNodes': 'false',
                'Mem/UseOdomGravity': 'true',
                'Optimizer/GravitySigma': '0.3',
                'Kp/MaxFeatures': '-1',
            }
        ],
        remappings=[
            ('scan_cloud', pointcloud_topic),
            ('odom', odom_topic),
            ('imu', imu_topic),
        ],
        arguments=['-d' if delete_db_on_start == 'true' else ''],
    )

    # 2. RTAB-Map Localization Mode Node
    rtabmap_localization_node = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'base_footprint',
                'database_path': database_path,
                'subscribe_imu': True,
                'Mem/IncrementalMemory': 'false',
                'Mem/InitWMWithAllNodes': 'true',
                'Mem/UseOdomGravity': 'true',
                'Optimizer/GravitySigma': '0.3',
                'Kp/MaxFeatures': '-1',
            }
        ],
        remappings=[
            ('scan_cloud', pointcloud_topic),
            ('odom', odom_topic),
            ('imu', imu_topic),
        ],
    )

    # 3. Optional RTAB-Map Visualization GUI Node
    rtabmap_viz_node = Node(
        condition=IfCondition(rtabmap_viz),
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'frame_id': 'base_footprint',
                'subscribe_imu': True,
            }
        ],
        remappings=[
            ('scan_cloud', pointcloud_topic),
            ('odom', odom_topic),
            ('imu', imu_topic),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_localization_cmd)
    ld.add_action(declare_pointcloud_topic_cmd)
    ld.add_action(declare_odom_topic_cmd)
    ld.add_action(declare_imu_topic_cmd)
    ld.add_action(declare_database_path_cmd)
    ld.add_action(declare_delete_db_on_start_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rtabmap_viz_cmd)

    ld.add_action(rtabmap_mapping_node)
    ld.add_action(rtabmap_localization_node)
    ld.add_action(rtabmap_viz_node)

    return ld
