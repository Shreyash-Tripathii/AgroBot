import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("agrobot_bringup")
    args = [
        DeclareLaunchArgument("ekf_params", default_value=os.path.join(pkg_share, "config", "ekf.yaml")),
        # measured with measure_imu_noise.py
        DeclareLaunchArgument("gyro_var", default_value="0"),
        DeclareLaunchArgument("accel_var", default_value="2.020e-03"),
    ]

    imu_driver = Node(
        package="agrobot_bringup",
        executable="bno085_ekf_driver.py",
        name="bno085_ekf_driver",
        output="screen",
        parameters=[{
            "gyro_var": ParameterValue(LaunchConfiguration("gyro_var"), value_type=float),
            "accel_var": ParameterValue(LaunchConfiguration("accel_var"), value_type=float),
        }],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[LaunchConfiguration("ekf_params")],
    )

    return LaunchDescription(args + [imu_driver, ekf])
