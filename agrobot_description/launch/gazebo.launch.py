import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('agrobot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'agrobot.urdf.xacro')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file for Gazebo Sim (Harmonic / ROS 2 Jazzy)'
    )

    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0', description='Initial robot X position')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0', description='Initial robot Y position')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.05', description='Initial robot Z position')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot Yaw angle')

    # Process URDF with ros2_control and simulation enabled
    robot_description_content = Command([
        'xacro ', xacro_file,
        ' use_ros2_control:=true',
        ' is_sim:=true',
        ' use_gazebo_plugins:=true'
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 1. Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 2. Gazebo Sim Launch (Harmonic on ROS 2 Jazzy) with Classic fallback
    launch_actions = []

    try:
        pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
        # ROS 2 Jazzy Standard: Gazebo Harmonic via ros_gz_sim
        start_gazebo_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': ['-r -v 1 ', world]}.items()
        )

        spawn_entity_node = Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'agrobot',
                '-x', x,
                '-y', y,
                '-z', z,
                '-Y', yaw
            ],
            output='screen'
        )

        # Bridge Gazebo topics to ROS 2 Jazzy topics
        ros_gz_bridge_node = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ],
            output='screen'
        )

        launch_actions.extend([start_gazebo_cmd, spawn_entity_node, ros_gz_bridge_node])

    except Exception:
        # Fallback for Classic Gazebo if ros_gz_sim is not installed
        pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
        start_gazebo_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world}.items()
        )
        spawn_entity_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'agrobot',
                '-x', x,
                '-y', y,
                '-z', z,
                '-Y', yaw
            ],
            output='screen'
        )
        launch_actions.extend([start_gazebo_cmd, spawn_entity_node])

    # 3. Spawners for ros2_control controllers
    spawner_args = ['--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30']

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'] + spawner_args,
    )

    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['agrobot_base_controller'] + spawner_args,
    )

    # Controller startup sequence
    load_jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    load_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner]
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(robot_state_pub_node)
    for act in launch_actions:
        ld.add_action(act)

    ld.add_action(load_jsb_after_spawn)
    ld.add_action(load_controller_after_jsb)

    return ld
