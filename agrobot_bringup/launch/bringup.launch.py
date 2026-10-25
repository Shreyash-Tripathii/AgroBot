import os

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Locate the agrobot_bringup package share directory
    pkg_agrobot_bringup = FindPackageShare('agrobot_bringup')

    # 1. Define Laser Launch
    laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_agrobot_bringup, 'launch', 'laser.launch.py'])
        )
    )

    # 2. Define Hardware Launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_agrobot_bringup, 'launch', 'hardware.launch.py'])
        )
    )

    # 3. Define Localization Launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_agrobot_bringup, 'launch', 'localization.launch.py'])
        )
    )

    # Create timers to enforce the startup sequence
    # Timer periods (in seconds) can be adjusted based on how long each system takes to initialize
    
    delayed_hardware_launch = TimerAction(
        period=3.0,  # Waits 3 seconds after launch starts before triggering hardware
        actions=[hardware_launch]
    )

    delayed_localization_launch = TimerAction(
        period=6.0,  # Waits 6 seconds after launch starts before triggering localization
        actions=[localization_launch]
    )

    yaw_heading_controller_node = Node(
        package='agrobot_bringup',          # <-- Replace with your package name
        executable='yaw_heading_controller.py',  # <-- Entry point name (or 'yaw_heading_controller.py' if using CMake)
        name='yaw_heading_controller',
        output='screen',
        parameters=[{
            'kp': 2.2,
            'kd': 0.15,
            'ki': 0.0,
            'max_angular_vel': 2.0,
            'max_heading_error': 0.785,
            'cmd_timeout': 0.5,
            'input_stamped': False,           # Set True if your teleop/Nav2 sends TwistStamped on /cmd_vel
            'frame_id': 'base_footprint',
        }],
        # Optional: if any node in your bringup was previously remapped from /cmd_vel 
        # to /agrobot_base_controller/cmd_vel, remove that remapping so it publishes to /cmd_vel instead!
    )

    delayed_yaw_controller = TimerAction(
	period=2.0,
	actions=[yaw_heading_controller_node]
    )

    return LaunchDescription([
        # Fire laser.launch.py immediately
        laser_launch,
        
        # Fire hardware.launch.py after the first timer finishes
        delayed_hardware_launch,
        
        # Fire localization.launch.py after the second timer finishes
        delayed_localization_launch,

        delayed_yaw_controller
    ])
