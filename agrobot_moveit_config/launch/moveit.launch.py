#!/usr/bin/env python3
"""
MoveIt 2 for the AgroBot camera arm: move_group only (no RViz).

  ros2 launch agrobot_moveit_config moveit.launch.py                      # real robot
  ros2 launch agrobot_moveit_config moveit.launch.py use_sim_time:=true   # with Gazebo

Normally you do not run this yourself: sim.launch.py and hardware.launch.py include it. The
planning UI (MotionPlanning display, drag the camera marker) lives in the single telemetry RViz:
  ros2 launch agrobot_telemetry telemetry.launch.py [sim:=true]

This launch only PLANS. It deliberately starts no robot_state_publisher, ros2_control or
controller spawners: those come from agrobot_bringup (hardware.launch.py) or agrobot_gazebo
(sim.launch.py), which already run `arm_controller` (FollowJointTrajectory, see
config/moveit_controllers.yaml). The robot description is the real agrobot.urdf.xacro, with the
xacro arguments recorded in .setup_assistant, so MoveIt always plans on the same model as the
rest of the stack.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("agrobot", package_name="agrobot_moveit_config")
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="true when running against Gazebo (agrobot_gazebo)"),
        move_group,
    ])
