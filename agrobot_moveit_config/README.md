# agrobot_moveit_config — MoveIt 2 for the AgroBot camera arm

Planning only. This package starts **move_group** only: no RViz, no robot_state_publisher, no
ros2_control, no controller spawners. Those come from
`agrobot_bringup` (real robot) or `agrobot_gazebo` (simulation), which already run
`arm_controller` (FollowJointTrajectory) — see `config/moveit_controllers.yaml`.

MoveIt plans on the real `agrobot_description/urdf/agrobot.urdf.xacro` (xacro args recorded in
`.setup_assistant`: `use_ros2_control:=false use_arm:=true use_imu:=false`), so it always matches
the rest of the stack. Group `arm`: `arm_base_link` → `camera_link`.

## Run

You normally do not launch this yourself: `agrobot_gazebo sim.launch.py` and
`agrobot_bringup hardware.launch.py` start move_group by default (`moveit:=false` to skip). The
planning UI is the MotionPlanning display of the single telemetry window:

```bash
ros2 launch agrobot_telemetry telemetry.launch.py sim:=true   # Gazebo
ros2 launch agrobot_telemetry telemetry.launch.py             # real robot (on a PC)
```

Standalone: `ros2 launch agrobot_moveit_config moveit.launch.py [use_sim_time:=true]`.

## Using it

* **Named poses** (`agrobot.srdf`, yaw / shoulder / wrist in rad): `home` 0 0 0, `look_down`
  0 0 0.55, `look_left` 1.2 0 0.2, `look_right` -1.2 0 0.2, `stowed` 0 -1.4 1.4. Pick them in the
  MotionPlanning panel (Goal State) or send a MoveGroup action goal.
* **IK is position-only** (pick_ik with `rotation_scale: 0.0`, `config/kinematics.yaml`): a 3-DOF
  arm cannot meet arbitrary 6-D poses, so dragging the camera marker moves the camera *to* the
  point and the orientation follows from the joints. Raise `rotation_scale` to weigh orientation;
  use the named poses / joint goals for exact camera angles.
* **Limits:** `config/joint_limits.yaml` caps every joint at 1.5 rad/s / 3 rad/s², matching the
  servo slew limit in `agrobot_arm_params.xacro`. Default scaling is 0.5.
* **Execution tolerances** in `moveit_controllers.yaml` are generous because the MG995 servos are
  open loop and the reported joint state is an estimate.
* Planners: OMPL (default) and Pilz (LIN/PTP/CIRC).

## Notes

* The two startup messages "Link imu_link/lidar_link has visual geometry but no collision
  geometry" and "No 3D sensor plugin(s) defined for octomap updates" are expected: sensors are not
  collision objects and the camera is RGB only (no octomap).
* Re-opening the MoveIt Setup Assistant (`ros2 launch agrobot_moveit_config setup_assistant.launch.py`)
  is safe. Do **not** run its "ROS 2 Controllers", "ros2_control URDF Modifications" or generate the
  demo launch again: the controllers live in `agrobot_bringup/config/controllers.yaml` and the
  hardware blocks in `agrobot_description`. After regenerating, delete anything it re-adds
  (`demo.launch.py`, `rsp.launch.py`, `spawn_controllers.launch.py`, `config/agrobot.urdf.xacro`,
  `config/agrobot.ros2_control.xacro`, `config/initial_positions.yaml`).
