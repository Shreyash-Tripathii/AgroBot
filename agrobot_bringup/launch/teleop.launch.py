import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('agrobot_bringup')
    joy_config = os.path.join(pkg_bringup, 'config', 'ps4.config.yaml')

    joy = LaunchConfiguration('joy')

    args = [
        DeclareLaunchArgument('joy', default_value='false',
                              description='true: DualShock4 via joy + teleop_twist_joy; '
                                          'false: teleop_twist_keyboard'),
        DeclareLaunchArgument('joy_device_id', default_value='0',
                              description='Index of the joystick (SDL device id)'),
        # teleop_twist_keyboard reads stdin, which launch does not forward, so it needs
        # its own terminal. Set to '' if you start this launch from a tty you will type in
        # (it will still not receive keys), or run the keyboard node manually instead.
#        DeclareLaunchArgument('keyboard_prefix', default_value='xterm -e',
 #                             description='Terminal wrapper for teleop_twist_keyboard'),
    ]

    # ---- Joystick path -------------------------------------------------------
    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'device_id': LaunchConfiguration('joy_device_id'),
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }],
        condition=IfCondition(joy),
    )

    teleop_joy = Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[joy_config],
        condition=IfCondition(joy),
    )   # publishes TwistStamped on /cmd_vel (publish_stamped_twist: true in the config)

    # ---- Keyboard path -------------------------------------------------------
    teleop_keyboard = Node(
        package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
#        prefix=LaunchConfiguration('keyboard_prefix'),
        parameters=[{'stamped': True, 'frame_id': 'base_link'}],
        condition=UnlessCondition(joy),
    )

    return LaunchDescription(args + [joy_node, teleop_joy, teleop_keyboard])
