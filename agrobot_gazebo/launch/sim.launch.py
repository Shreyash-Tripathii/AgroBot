#!/usr/bin/env python3
"""
Gazebo (gz sim) simulation of AgroBot - one command, same stack as the real robot.

  ros2 launch agrobot_gazebo sim.launch.py                       # crop field + Gazebo GUI
  ros2 launch agrobot_gazebo sim.launch.py gui:=false            # headless
  ros2 launch agrobot_telemetry telemetry.launch.py sim:=true    # the ONE RViz window (separate)
  ros2 launch agrobot_gazebo sim.launch.py world:=empty_test use_arm:=false

What runs (names, topics and frames match hardware.launch.py + laser.launch.py):
  gz sim + world         gz_ros2_control (GazeboSimSystem) hosts the controller manager
  controllers            joint_state_broadcaster, imu_game/imu_mag broadcasters,
                         [agrobot_base_controller <- yaw_heading_controller] chain, arm controllers
  sensors -> ROS         /scan_raw (-> box filter -> /scan), /gps/fix, /camera/image_raw,
                         /camera/camera_info, /clock; IMUs via ros2_control like the real BNO085
  localisation           robot_localization EKF (same ekf.yaml)

This launch starts NO RViz: visualisation is detached (agrobot_telemetry). move_group (MoveIt) runs
here by default when use_arm is true; pass moveit:=false to skip it.

Drive it with any /cmd_vel publisher (plain Twist), e.g.
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
Navigation: ros2 launch agrobot_navigation autonomy.launch.py mode:=mapping sim:=true
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() == 'true'


def launch_setup(context, *args, **kwargs):
    pkg_gazebo = get_package_share_directory('agrobot_gazebo')
    pkg_desc = get_package_share_directory('agrobot_description')
    pkg_bringup = get_package_share_directory('agrobot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_arm = _flag(context, 'use_arm')
    use_ekf = _flag(context, 'use_ekf')
    use_moveit = _flag(context, 'moveit') and use_arm
    gui = _flag(context, 'gui')
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    world = LaunchConfiguration('world').perform(context).strip()
    world_path = world if world.endswith('.sdf') else os.path.join(pkg_gazebo, 'worlds', f'{world}.sdf')
    if not os.path.isfile(world_path):
        raise RuntimeError(f'World not found: {world_path}')

    sim_time = {'use_sim_time': True}
    models_dir = os.path.join(pkg_gazebo, 'models')
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = os.pathsep.join([p for p in (models_dir, os.path.join(pkg_gazebo, 'worlds'),
                                                 existing) if p])

    # ---- Gazebo ------------------------------------------------------------
    gz_flags = '-r -v2' if gui else '-r -v2 -s --headless-rendering'
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{gz_flags} {world_path}', 'on_exit_shutdown': 'true'}.items(),
    )

    # ---- robot description (sim variant of the SAME xacro) -----------------------
    xacro_file = os.path.join(pkg_desc, 'urdf', 'agrobot.urdf.xacro')
    controllers = os.path.join(pkg_bringup, 'config', 'controllers.yaml')
    override = os.path.join(pkg_gazebo, 'config', 'controllers_sim.yaml')
    robot_description = {'robot_description': ParameterValue(Command([
        'xacro ', xacro_file, ' sim_mode:=true',
        ' use_arm:=', 'true' if use_arm else 'false',
        ' sim_controllers:=', controllers,
        ' sim_controllers_override:=', override,
    ]), value_type=str)}

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
               output='screen', parameters=[robot_description, sim_time])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'agrobot',
                            '-x', x, '-y', y, '-z', z, '-Y', yaw])

    # ---- gz <-> ROS bridges ------------------------------------------------------
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  parameters=[{'config_file': os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')},
                              sim_time])
    image_bridge = Node(package='ros_gz_image', executable='image_bridge', output='screen',
                        arguments=['camera/image_raw'], parameters=[sim_time])

    # ---- controllers (same set as hardware.launch.py, minus the ESP32-only scheduler) ----
    base = ['--controller-manager', '/controller_manager', '--controller-manager-timeout', '120']

    def spawner(names, extra=()):
        return Node(package='controller_manager', executable='spawner',
                    arguments=list(names) + base + list(extra), parameters=[sim_time])

    controller_nodes = [
        spawner(['joint_state_broadcaster']),
        spawner(['imu_game_broadcaster'], ['--controller-ros-args', '-r ~/imu:=/imu/data_game']),
        spawner(['imu_mag_broadcaster'], ['--controller-ros-args', '-r ~/imu:=/imu/data_mag']),
        spawner(['agrobot_base_controller', 'yaw_heading_controller'], ['--activate-as-group']),
    ]
    if use_arm:
        controller_nodes += [spawner(['arm_controller']),
                             spawner(['arm_position_controller'], ['--inactive'])]

    # ---- lidar arm mask (identical to laser.launch.py) + EKF + RViz -------------------
    scan_filter = Node(
        package='laser_filters', executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain', output='screen',
        parameters=[os.path.join(pkg_bringup, 'config', 'laser_filter.yaml'), sim_time],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')])

    actions = [SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
               gz_sim, rsp,
               # give the gz server time to come up before asking it to spawn
               TimerAction(period=4.0, actions=[spawn]),
               bridge, scan_filter] + controller_nodes
    if use_arm:
        actions.append(image_bridge)
    if use_ekf:
        actions.append(Node(package='robot_localization', executable='ekf_node',
                            name='ekf_filter_node', output='screen',
                            parameters=[os.path.join(pkg_bringup, 'config', 'ekf.yaml'), sim_time]))
    if use_moveit:
        # Planning only (move_group): the controllers and robot_state_publisher above already
        # serve it. The planning UI is the MotionPlanning display of agrobot_telemetry.
        try:
            moveit_launch = os.path.join(
                get_package_share_directory('agrobot_moveit_config'), 'launch', 'moveit.launch.py')
        except PackageNotFoundError:
            actions.append(LogInfo(msg='[sim] agrobot_moveit_config not found: MoveIt disabled '
                                       '(build it, or pass moveit:=false).'))
        else:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={'use_sim_time': 'true'}.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='agro_field',
                              description="'agro_field', 'empty_test', or a path to an .sdf"),
        DeclareLaunchArgument('use_arm', default_value='true',
                              description='Include the 3-DOF arm and its camera'),
        DeclareLaunchArgument('use_ekf', default_value='true'),
        DeclareLaunchArgument('moveit', default_value='true',
                              description='Start MoveIt move_group for the arm (needs use_arm)'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false = headless server (no gz window)'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.05'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])
