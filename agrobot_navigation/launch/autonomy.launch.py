#!/usr/bin/env python3
"""
autonomy.launch.py

Unified master launch file for agrobot_navigation:
  - mode:=mapping       (SLAM Toolbox + auto-save on Ctrl+C)
  - explore:=true       (SLAM Toolbox + Nav2 + Frontier Explorer + auto-save & exit)
  - mode:=localization  (Map Server + AMCL + global_localizer spin + Nav2)
  - mode:=gps           (u-blox NEO-M8N + navsat_transform + Global EKF + Rolling-Window Nav2)
  - sim:=true           (Gazebo: use_sim_time, maps in maps/sim/, Gazebo's /gps/fix instead of the
                         u-blox driver, sim_overrides.yaml for navsat). Combine with any mode:
                         ros2 launch agrobot_gazebo sim.launch.py  +  autonomy.launch.py sim:=true
"""

import os
import glob
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_maps_directory(pkg_name: str = 'agrobot_navigation') -> str:
    share_dir = get_package_share_directory(pkg_name)
    ws_root = os.path.abspath(os.path.join(share_dir, '..', '..', '..', '..'))
    src_candidates = glob.glob(
        os.path.join(ws_root, 'src', '**', pkg_name), recursive=True
    )
    if src_candidates:
        maps_dir = os.path.join(src_candidates[0], 'maps')
    else:
        maps_dir = os.path.join(share_dir, 'maps')

    os.makedirs(maps_dir, exist_ok=True)
    return maps_dir


def resolve_latest_map_yaml(maps_dir: str, requested_name: str) -> str:
    if requested_name:
        base = requested_name[:-5] if requested_name.endswith('.yaml') else requested_name
        candidate = base if os.path.isabs(base) else os.path.join(maps_dir, base)
        return candidate + '.yaml'

    pointer_file = os.path.join(maps_dir, 'latest_map.txt')
    if os.path.isfile(pointer_file):
        with open(pointer_file, 'r', encoding='utf-8') as f:
            last_name = f.read().strip()
        if last_name:
            candidate = os.path.join(maps_dir, last_name + '.yaml')
            if os.path.isfile(candidate):
                return candidate

    yaml_files = glob.glob(os.path.join(maps_dir, '*.yaml'))
    if not yaml_files:
        raise RuntimeError(
            f"No maps found in '{maps_dir}' and no map_name was provided! "
            f"Run with mode:=mapping or explore:=true first."
        )
    return max(yaml_files, key=os.path.getmtime)


def build_nav2_core_nodes(param_files: list, use_sim_time: bool, include_waypoint_follower: bool = True,
                          use_emra: bool = False):
    """Helper to avoid repeating Nav2 server declarations across modes.

    use_emra=False -> stock Nav2 setup exactly as configured in nav2_params.yaml.
    use_emra=True  -> EMRA global planner, event-driven replanning BTs and chassis footprint on top.
    """
    bt_params = {'use_sim_time': use_sim_time}
    if use_emra:
        param_files = param_files + [os.path.join(
            get_package_share_directory('agrobot_navigation'), 'config', 'emra_overrides.yaml')]
        emra_share = get_package_share_directory('agrobot_emra_planner')
        bt_params.update({
            'default_nav_to_pose_bt_xml': os.path.join(emra_share, 'behavior_trees', 'navigate_to_pose_emra.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                emra_share, 'behavior_trees', 'navigate_through_poses_emra.xml'),
        })
    nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=param_files + [{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel', '/cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=param_files + [{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=param_files + [{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=param_files + [{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=param_files + [bt_params],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=param_files + [{'use_sim_time': use_sim_time}],
            remappings=[
                ('/cmd_vel', '/cmd_vel_nav'),
                ('/cmd_vel_smoothed', '/cmd_vel'),
            ],
        ),
    ]

    managed_names = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'velocity_smoother',
    ]

    if include_waypoint_follower:
        nodes.append(
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                parameters=param_files + [{'use_sim_time': use_sim_time}],
            )
        )
        managed_names.insert(5, 'waypoint_follower')

    nodes.append(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': managed_names,
            }],
        )
    )
    return nodes


def launch_setup(context, *args, **kwargs):
    pkg_name = 'agrobot_navigation'
    share_dir = get_package_share_directory(pkg_name)
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    map_name_arg = LaunchConfiguration('map_name').perform(context).strip()
    explore_arg = LaunchConfiguration('explore').perform(context).strip().lower()
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context).strip().lower()

    sim = LaunchConfiguration('sim').perform(context).strip().lower() in ('true', '1', 'yes')

    explore = explore_arg in ('true', '1', 'yes')
    use_emra = LaunchConfiguration('use_emra').perform(context).strip().lower() in ('true', '1', 'yes')
    use_sim_time = use_sim_time_str in ('true', '1', 'yes') or sim

    maps_dir = resolve_maps_directory(pkg_name)
    if sim:
        # Simulated maps never touch the real robot's maps / latest_map.txt
        maps_dir = os.path.join(maps_dir, 'sim')
        os.makedirs(maps_dir, exist_ok=True)

    if explore:
        mode = 'mapping'

    nav2_params_path = os.path.join(share_dir, 'config', 'nav2_params.yaml')
    slam_params_path = os.path.join(share_dir, 'config', 'slam_toolbox_params.yaml')
    rl_params_path = os.path.join(share_dir, 'config', 'dual_ekf_navsat.yaml')
    ublox_params_path = os.path.join(share_dir, 'config', 'ublox_gps.yaml')
    sim_params_path = os.path.join(share_dir, 'config', 'sim_overrides.yaml')
    # Later files win: sim_overrides.yaml adjusts navsat/EKF for Gazebo's ENU world
    rl_params = [rl_params_path] + ([sim_params_path] if sim else [])

    actions = []

    # =========================================================================
    # MODE A: MAPPING (Manual or Autonomous Exploration)
    # =========================================================================
    if mode == 'mapping':
        if map_name_arg:
            clean_name = (
                map_name_arg[:-5] if map_name_arg.endswith('.yaml') else map_name_arg
            )
        else:
            clean_name = f"map_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

        map_save_prefix = os.path.join(maps_dir, clean_name)
        slam_launch_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')

        actions.append(
            LogInfo(
                msg=f"[Autonomy] Starting MAPPING mode (explore={explore}). "
                    f"Map will be saved to: {map_save_prefix}.yaml/.pgm"
            )
        )

        slam_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_launch_dir, 'online_async_launch.py')
            ),
            launch_arguments={
                'slam_params_file': slam_params_path,
                'use_sim_time': str(use_sim_time),
            }.items(),
        )
        actions.append(slam_launch)

        autosaver_node = Node(
            package=pkg_name,
            executable='map_autosaver.py',
            name='map_autosaver',
            output='screen',
            parameters=[{
                'map_save_prefix': map_save_prefix,
                'maps_dir': maps_dir,
                'map_name': clean_name,
            }],
        )
        actions.append(autosaver_node)

        actions.append(
            RegisterEventHandler(
                OnShutdown(
                    on_shutdown=[
                        LogInfo(msg=f"[Autonomy] Saving map '{clean_name}' before exit..."),
                    ]
                )
            )
        )

        if explore:
            actions.extend(
                build_nav2_core_nodes([nav2_params_path], use_sim_time, include_waypoint_follower=False,
                                      use_emra=use_emra)
            )

            explorer_node = Node(
                package=pkg_name,
                executable='frontier_explorer.py',
                name='frontier_explorer',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'min_frontier_size': 8,
                    'goal_timeout_s': 45.0,
                }],
            )
            actions.append(TimerAction(period=6.0, actions=[explorer_node]))

            actions.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=explorer_node,
                        on_exit=[
                            LogInfo(msg='[Autonomy] Exploration complete & map saved. Exiting.'),
                            EmitEvent(event=Shutdown(reason='Exploration finished')),
                        ],
                    )
                )
            )

    # =========================================================================
    # MODE B: INDOOR LOCALIZATION + NAV2 (AMCL + Global Localizer)
    # =========================================================================
    elif mode == 'localization':
        map_yaml_path = resolve_latest_map_yaml(maps_dir, map_name_arg)
        if not os.path.isfile(map_yaml_path):
            raise RuntimeError(f"Map file not found: {map_yaml_path}")

        actions.append(
            LogInfo(msg=f"[Autonomy] Starting LOCALIZATION mode using map: {map_yaml_path}")
        )

        localization_nodes = [
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[
                    nav2_params_path,
                    {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml_path},
                ],
            ),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                parameters=[nav2_params_path, {'use_sim_time': use_sim_time}],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': ['map_server', 'amcl'],
                }],
            ),
        ]

        global_localizer_node = Node(
            package=pkg_name,
            executable='global_localizer.py',
            name='global_localizer',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        )

        actions.extend(localization_nodes)
        actions.extend(build_nav2_core_nodes([nav2_params_path], use_sim_time, use_emra=use_emra))
        actions.append(TimerAction(period=3.0, actions=[global_localizer_node]))

    # =========================================================================
    # MODE C: OUTDOOR GPS NAVIGATION (NEO-M8N + NavSat + Global EKF + Nav2)
    # =========================================================================
    elif mode == 'gps':
        actions.append(
            LogInfo(
                msg="[Autonomy] Starting OUTDOOR GPS mode (u-blox NEO-M8N + NavSat + Rolling Global Costmap)"
            )
        )

        gps_and_ekf_nodes = [] if sim else [
            # 1. u-blox NEO-M8N UART Driver (in simulation Gazebo publishes /gps/fix itself)
            Node(
                package='ublox_gps',
                executable='ublox_gps_node',
                name='ublox_gps_node',
                output='screen',
                parameters=[ublox_params_path],
                remappings=[('/fix', '/gps/fix')],
            ),
        ]
        gps_and_ekf_nodes += [
            # 2. Fallback static TF base_footprint -> gps_link (remove if already in URDF)
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_gps_tf',
                arguments=['0.0', '0.0', '0.25', '0.0', '0.0', '0.0', 'base_footprint', 'gps_link'],
            ),
            # 3. Global EKF (publishes map -> odom)
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node_map',
                output='screen',
                parameters=rl_params + [{'use_sim_time': use_sim_time}],
                remappings=[('odometry/filtered', '/odometry/global')],
            ),
            # 4. NavSat Transform (fuses /imu/data_mag + /gps/fix + /odometry/global -> /odometry/gps)
            Node(
                package='robot_localization',
                executable='navsat_transform_node',
                name='navsat_transform_node',
                output='screen',
                parameters=rl_params + [{'use_sim_time': use_sim_time}],
                remappings=[
                    ('imu', '/imu/data_mag'),
                    ('gps/fix', '/gps/fix'),
                    ('odometry/filtered', '/odometry/global'),
                    ('odometry/gps', '/odometry/gps'),
                ],
            ),
        ]

        actions.extend(gps_and_ekf_nodes)
        # Pass rl_params_path SECOND so its rolling-window global_costmap overrides nav2_params.yaml
        actions.extend(
            build_nav2_core_nodes([nav2_params_path, rl_params_path], use_sim_time, use_emra=use_emra)
        )

    else:
        raise RuntimeError(
            f"Unknown mode '{mode}'. Valid options: 'mapping', 'localization', or 'gps'."
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='localization',
            description="Operation mode: 'mapping', 'localization', or 'gps'",
        ),
        DeclareLaunchArgument(
            'map_name',
            default_value='',
            description='Map name (without .yaml). Auto-generated in mapping if empty; '
                        'uses last saved map in localization if empty.',
        ),
        DeclareLaunchArgument(
            'explore',
            default_value='false',
            description='If true, automatically runs SLAM + Frontier Exploration, '
                        'saves map on completion, and exits.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_emra',
            default_value='false',
            description='true = EMRA global planner + event-driven replanning BT + chassis footprint '
                        '(config/emra_overrides.yaml); false = stock Nav2 setup (NavFn, default BT)',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Gazebo simulation: implies use_sim_time, keeps maps in maps/sim/, '
                        'skips the u-blox driver and applies config/sim_overrides.yaml',
        ),
        OpaqueFunction(function=launch_setup),
    ])
