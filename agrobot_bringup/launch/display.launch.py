import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue # <-- Imported this

def generate_launch_description():

    pkg_description = get_package_share_directory('agrobot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'agrobot.urdf.xacro')
    rviz_config_file = os.path.join(pkg_description, 'rviz', 'agrobot_view.rviz')

    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Whether to load the ros2_control tags in the URDF'
    )

    robot_description_content = Command(
        ['xacro ', xacro_file, ' use_ros2_control:=', LaunchConfiguration('use_ros2_control')]
    )

    # <-- Wrapped the Command output in ParameterValue(..., value_type=str)
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        use_ros2_control_arg,
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])
