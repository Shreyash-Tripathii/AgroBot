#!/usr/bin/env python3
"""
AgroBot telemetry: ONE RViz window for everything, for the simulation and the real robot.

  ros2 launch agrobot_telemetry telemetry.launch.py sim:=true   # next to agrobot_gazebo sim.launch.py
  ros2 launch agrobot_telemetry telemetry.launch.py             # next to hardware.launch.py (on a PC)

Shows: robot + arm model, EKF/wheel/GPS odometry, lidar, the arm camera, SLAM/AMCL map, costmaps,
plans, AMCL particles, the Navigation 2 panel (goals, waypoints, cancel) and the MoveIt
MotionPlanning display (plan/execute, named poses, drag the camera marker). RViz only: the robot
side (sim.launch.py or hardware.launch.py) runs ros2_control, the EKF and MoveIt's move_group.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # The MotionPlanning display needs the same model/kinematics/pipelines as move_group.
    moveit_config = (
        MoveItConfigsBuilder("agrobot", package_name="agrobot_moveit_config")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="agrobot_telemetry",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": ParameterValue(LaunchConfiguration("sim"), value_type=bool)},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false",
                              description="true when the robot side is the Gazebo simulation"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("agrobot_telemetry"), "rviz", "agrobot_telemetry.rviz"]),
            description="Override to use your own layout"),
        rviz,
    ])
