#!/usr/bin/env python3
"""
Single entry point for everything hardware-related on the robot RPi (the one without
the lidar): ros2_control with the ESP32 drive/arm, the BNO085 IMU, all controllers
(IMU broadcasters -> heading hold -> fuzzy gain scheduler -> diff drive, plus the arm)
and the EKF.

  ros2 launch agrobot_bringup hardware.launch.py
  ros2 launch agrobot_bringup hardware.launch.py use_arm:=false mock_imu:=true  # bench test
  ros2 launch agrobot_bringup hardware.launch.py moveit:=false   # without MoveIt move_group

No RViz here. With use_arm (default) this also runs MoveIt's move_group (planning); it is skipped
with a warning if MoveIt / agrobot_moveit_config is not installed on this machine. View and drive
everything from a PC with ONE window:
  ros2 launch agrobot_telemetry telemetry.launch.py     (same ROS_DOMAIN_ID / DDS settings)

ESP32 port: with esp32_port:=auto (default) every /dev/ttyUSB* and /dev/ttyACM* is
scanned. A single port is the ESP32; with several, the one whose USB vendor is a known
ESP32 bridge (CP210x, CH34x, FTDI, Espressif) wins. The chosen port is linked to
/tmp/esp32, which the URDF uses. Pass esp32_port:=/dev/ttyUSB1 to force one.
"""
import glob
import os
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.events import Shutdown
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ESP32_LINK = '/tmp/esp32'
# USB vendor ids of the usual ESP32 dev-board bridges: CP210x, CH340/CH9102, FTDI, Espressif
ESP32_USB_VENDORS = {'10c4', '1a86', '0403', '303a'}


# --------------------------------------------------------------------------- port detection
def list_serial_ports():
    return sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*'))


def usb_vendor_id(port):
    """USB idVendor of a tty device (walks up sysfs to the USB device), or None."""
    path = os.path.realpath(os.path.join('/sys/class/tty', os.path.basename(port), 'device'))
    while path and path != '/':
        candidate = os.path.join(path, 'idVendor')
        if os.path.isfile(candidate):
            try:
                with open(candidate) as f:
                    return f.read().strip().lower()
            except OSError:
                return None
        path = os.path.dirname(path)
    return None


def choose_esp32_port(ports, vendor_fn=usb_vendor_id):
    """One port -> it. Several -> the single one with a known ESP32 vendor id, else None."""
    if len(ports) == 1:
        return ports[0]
    known = [p for p in ports if vendor_fn(p) in ESP32_USB_VENDORS]
    return known[0] if len(known) == 1 else None


def find_esp32_port(timeout, list_fn=list_serial_ports, vendor_fn=usb_vendor_id,
                    sleep=time.sleep, now=time.monotonic):
    """Polls until a port can be chosen or the timeout passes. Returns (port|None, ports)."""
    deadline = now() + timeout
    while True:
        ports = list_fn()
        port = choose_esp32_port(ports, vendor_fn)
        if port or now() >= deadline:
            return port, ports
        sleep(0.5)  # devices may still be enumerating at boot


def link_port(link, target):
    """Atomically (re)point `link` at `target`. Returns None on success or an error string."""
    tmp = f'{link}.tmp{os.getpid()}'
    try:
        if os.path.lexists(tmp):
            os.remove(tmp)
        os.symlink(target, tmp)
        os.replace(tmp, link)
        return None
    except OSError as e:
        if os.path.lexists(tmp):
            os.remove(tmp)
        return str(e)


# --------------------------------------------------------------------------- launch
def _flag(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() == 'true'


def launch_setup(context, *args, **kwargs):
    use_arm = _flag(context, 'use_arm')
    use_imu = _flag(context, 'use_imu')
    mock_imu = _flag(context, 'mock_imu')
    use_ekf = _flag(context, 'use_ekf')
    use_moveit = _flag(context, 'moveit') and use_arm
    i2c_bus = LaunchConfiguration('i2c_bus').perform(context)
    requested = LaunchConfiguration('esp32_port').perform(context).strip()
    timeout = float(LaunchConfiguration('port_timeout').perform(context))
    ekf_params = LaunchConfiguration('ekf_params').perform(context)

    actions = []

    # ---- ESP32 serial port -> /tmp/esp32 ---------------------------------------
    if requested.lower() == 'auto':
        port, ports = find_esp32_port(timeout)
        if port is None:
            why = ('no /dev/ttyUSB* or /dev/ttyACM* device found' if not ports else
                   f'several serial ports found {ports} and none (or more than one) has a '
                   'known ESP32 USB vendor id; pass esp32_port:=<device>')
            return [
                LogInfo(msg=f'[hardware] ERROR: cannot locate the ESP32: {why}.'),
                EmitEvent(event=Shutdown(reason='ESP32 serial port not found')),
            ]
    else:
        port = requested
    if os.path.realpath(port) != os.path.realpath(ESP32_LINK):
        err = link_port(ESP32_LINK, port)
        if err:
            return [
                LogInfo(msg=f'[hardware] ERROR: cannot link {ESP32_LINK} -> {port}: {err}'),
                EmitEvent(event=Shutdown(reason='cannot create /tmp/esp32 link')),
            ]
        actions.append(LogInfo(msg=f'[hardware] ESP32 on {port} ({ESP32_LINK} -> {port})'))
    else:
        actions.append(LogInfo(msg=f'[hardware] using {ESP32_LINK} directly'))

    # ---- robot description + controller manager ---------------------------------
    pkg_description = get_package_share_directory('agrobot_description')
    pkg_bringup = get_package_share_directory('agrobot_bringup')
    xacro_file = os.path.join(pkg_description, 'urdf', 'agrobot.urdf.xacro')
    controller_config = os.path.join(pkg_bringup, 'config', 'controllers.yaml')

    robot_description_content = Command([
        'xacro ', xacro_file, ' use_ros2_control:=true',
        ' serial_port:=', ESP32_LINK,
        ' use_arm:=', 'true' if use_arm else 'false',
        ' use_imu:=', 'true' if use_imu else 'false',
        ' mock_imu:=', 'true' if mock_imu else 'false',
        ' i2c_bus:=', i2c_bus,
    ])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str)}

    actions.append(Node(
        package='controller_manager', executable='ros2_control_node', output='screen',
        parameters=[robot_description, controller_config],
        remappings=[('~/robot_description', '/robot_description')],
    ))
    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[robot_description],
    ))

    # ---- controllers (spawners run concurrently; the manager serialises them) -----
    base = ['--controller-manager', '/controller_manager', '--controller-manager-timeout', '30']

    def spawner(names, extra=()):
        return Node(package='controller_manager', executable='spawner',
                    arguments=list(names) + base + list(extra))

    actions.append(spawner(['joint_state_broadcaster']))
    if use_imu:
        actions.append(spawner(['imu_game_broadcaster'],
                               ['--controller-ros-args', '-r ~/imu:=/imu/data_game']))
        actions.append(spawner(['imu_mag_broadcaster'],
                               ['--controller-ros-args', '-r ~/imu:=/imu/data_mag']))
    # Drive chain, tail -> head, activated together so the chain is built in one switch.
    actions.append(spawner(
        ['agrobot_base_controller', 'fuzzy_gain_scheduler', 'yaw_heading_controller'],
        ['--activate-as-group']))
    if use_arm:
        # Trajectory controller active for MoveIt; the raw position controller is loaded
        # inactive (switch with `ros2 control switch_controllers`).
        actions.append(spawner(['arm_controller']))
        actions.append(spawner(['arm_position_controller'], ['--inactive']))

    # ---- EKF (wheel odom + IMU) --------------------------------------------------
    if use_ekf:
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node',
            output='screen', parameters=[ekf_params],
        ))
    # ---- MoveIt move_group (planning only; arm_controller above executes the trajectories) ----
    if use_moveit:
        try:
            moveit_launch = os.path.join(
                get_package_share_directory('agrobot_moveit_config'), 'launch', 'moveit.launch.py')
        except PackageNotFoundError:
            actions.append(LogInfo(msg='[hardware] agrobot_moveit_config not found: MoveIt '
                                       'disabled (install MoveIt, or pass moveit:=false).'))
        else:
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(moveit_launch),
                launch_arguments={'use_sim_time': 'false'}.items()))
    return actions


def generate_launch_description():
    pkg_bringup = get_package_share_directory('agrobot_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('use_arm', default_value='true',
                              description='Include the 3-DOF camera arm (servo joints + controllers)'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Include the BNO085 IMU (hardware, broadcasters, heading hold)'),
        DeclareLaunchArgument('mock_imu', default_value='false',
                              description='Replace the BNO085 with a mock sensor (bench/PC testing)'),
        DeclareLaunchArgument('moveit', default_value='true',
                              description='Run MoveIt move_group for the arm (needs use_arm and '
                                          'MoveIt + agrobot_moveit_config; skipped if missing)'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='Start the robot_localization EKF (wheel odom + IMU)'),
        DeclareLaunchArgument('ekf_params',
                              default_value=os.path.join(pkg_bringup, 'config', 'ekf.yaml')),
        DeclareLaunchArgument('i2c_bus', default_value='3', description='BNO085 I2C bus number'),
        DeclareLaunchArgument('esp32_port', default_value='auto',
                              description="'auto' = scan ttyUSB*/ttyACM*, or a device path"),
        DeclareLaunchArgument('port_timeout', default_value='10',
                              description='Seconds to wait for the ESP32 to enumerate'),
        OpaqueFunction(function=launch_setup),
    ])
