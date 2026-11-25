import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    pkg_description = get_package_share_directory('agrobot_description')
    pkg_bringup = get_package_share_directory('agrobot_bringup')

    xacro_file = os.path.join(pkg_description, 'urdf', 'agrobot.urdf.xacro')
    controller_config = os.path.join(pkg_bringup, 'config', 'controllers.yaml')

    # Camera arm on/off (adds 3 servo joints to the same ESP32 hardware + spawns its controllers)
    # Resolved here (not via IfCondition) because the OnProcessExit handlers below run
    # later, outside this launch file's argument scope when it is included from bringup.
    use_arm = LaunchConfiguration('use_arm').perform(context).strip().lower() == 'true'

    # Process URDF with ros2_control enabled
    robot_description_content = Command(
        ['xacro ', xacro_file, ' use_ros2_control:=true use_arm:=', 'true' if use_arm else 'false'])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 1. Start the Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
        remappings=[
            ('~/robot_description', '/robot_description'),
        ]
    )

    # 2. Publish TF tree
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    spawner_args = ['--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30']

    # 3. Spawn the joint state broadcaster (reads from hardware plugin)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'] + spawner_args,
    )

    # 4. Spawn the drive controller (writes velocity to hardware plugin)
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['agrobot_base_controller'] + spawner_args,
    )

    # 5. Spawn the PID gain controllers (write kp/ki/kd to hardware plugin)
    gain_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['kp_controller', 'ki_controller', 'kd_controller'] + spawner_args,
    )

    # 6. Adaptive gain scheduler (5 Hz outer loop)
    # If installed via install(PROGRAMS ...) the executable keeps the .py suffix;
    # if you registered it as a console_scripts entry point, drop the suffix.
    gain_scheduler = Node(
        package='agrobot_bringup',
        executable='gain_scheduler_node.py',
        name='gain_scheduler',
        output='screen',
    )

    # 7. Camera arm controllers (optional). Trajectory controller active for MoveIt; the
    # raw position controller is loaded inactive (switch with `ros2 control switch_controllers`).
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'] + spawner_args,
    )
    arm_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_position_controller', '--inactive'] + spawner_args,
    )

    arm_handlers = []
    if use_arm:
        arm_handlers = [
            # Arm controllers: independent of the base chain, started once the JSB is up
            RegisterEventHandler(OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[arm_controller_spawner])),
            RegisterEventHandler(OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[arm_position_controller_spawner])),
        ]

    return [
        controller_manager,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        # Chain: JSB -> drive controller -> gain controllers -> scheduler
        RegisterEventHandler(OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[gain_controllers_spawner])),
        RegisterEventHandler(OnProcessExit(
            target_action=gain_controllers_spawner,
            on_exit=[gain_scheduler])),
        *arm_handlers,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_arm', default_value='true',
            description='Include the 3-DOF camera arm (URDF, servo joints, arm controllers)'),
        OpaqueFunction(function=launch_setup),
    ])
