# agrobot_telemetry — telemetry for AgroBot

Two front ends on the same robot data. Both are view-only: the robot side (`sim.launch.py` or
`hardware.launch.py`) runs ros2_control, the EKF and MoveIt's `move_group`. On the real robot use the
same `ROS_DOMAIN_ID` / DDS settings on the PC and the RPi.

| | RViz (`telemetry.launch.py`) | Foxglove Studio (`foxglove.launch.py`) |
|---|---|---|
| Best for | arm motion planning (drag the camera marker), Nav2 goals | live plots, health, drive pad, remote viewing, recording |
| Shows | map, scan, camera, robot, MoveIt planning | all of that plus plots, diagnostics, logs, GPS map |

## Foxglove Studio

```bash
ros2 launch agrobot_telemetry foxglove.launch.py sim:=true studio:=true   # with the Gazebo sim
ros2 launch agrobot_telemetry foxglove.launch.py studio:=true             # real robot
```

Starts `foxglove_bridge` (websocket `ws://<host>:8765`), `telemetry_monitor`, `arm_commander` and optionally opens
Studio connected to it. **First time only:** Foxglove cannot preload a layout from a link, so in
Studio use the *Layouts* menu (top right) → *Import from file…* and pick one of
`foxglove/layouts/` (installed under `share/agrobot_telemetry/foxglove/layouts/`):

* `agrobot_mission_control.json` — one dashboard: 3D (robot, map, costmap, scan, plans; click to set
  goals / initial pose), Drive pad (publishes `/cmd_vel`), speed and yaw-rate plots (command vs
  measured), Health (diagnostics), the arm camera and, below it, the **Arm Planner** panel (see below).
* `agrobot_engineering.json` — tuning: wheel velocities, arm feedback/reference/error, EKF position,
  the TF tree (two cells wide), a robot-only 3D view that follows `/joint_states`, and a **LiDAR
  control window** (below).

### Autonomy control (Engineering layout, middle right; also `agrobot_autonomy.json`)

`autonomy_manager` (started by `foxglove.launch.py`, `autonomy_manager:=false` to skip) runs
`agrobot_navigation/autonomy.launch.py` for you. Buttons: **Explore** (mapping + frontier explorer),
**Mapping**, **Localization**, **GPS**, and **Shut down autonomy** (Ctrl-C, so mapping still saves its
map; a button for another mode first shuts the running one down). Panels: state + mode indicators, the
lifecycle state of every lifecycle node it started (`active` green, `inactive` / `unconfigured` yellow,
missing red), diagnostics of the whole launch (one entry per node, plus the launch itself) and its live
log. Services: `/autonomy_manager/{start_explore,start_mapping,start_localization,start_gps,stop}`;
parameters `map_name`, `extra_args` (e.g. `use_emra:=true`), `sim`.

### LiDAR control (Engineering layout, bottom right)

`telemetry.launch.py` (2nd RPi) runs `lidar_manager`, which owns `sllidar_node`: status indicator
(running / starting / stopped / CRASHED), the "LiDAR driver" diagnostic (port, pid, uptime, restarts,
last exit code, last error line), the driver's live log (including what it prints while crashing, which
never reaches `/rosout`) and **Start / Relaunch / Stop** buttons (services `/lidar_manager/{start,restart,stop}`).
After a crash it restarts the driver by itself up to 5 times in a row, then waits for the button
(`lidar_auto_restart:=false` to always wait). Stop is final until you press Start.

### Arm Planner panel (MoveIt-style, custom extension)

A Foxglove extension in `foxglove/extension/` adds an **Arm Planner** panel: the arm model, a
draggable orange ball end effector, your SRDF presets, Plan / Execute / Plan & Execute / Stop.
Install once per machine (Studio loads extensions at start-up, so restart it afterwards):

```bash
sudo apt install nodejs npm
cd <agrobot_telemetry>/foxglove/extension     # the source tree, not install/
npm install && npm run local-install          # -> ~/.foxglove-studio/extensions/
# npm run package   builds a .foxe you can install on another PC
```

Then re-import `agrobot_mission_control.json` (the panel type is `AgroBotArmPlanner.Arm Planner`).
It needs `move_group` (started by the sim / hardware launch) and `arm_commander`, which
`foxglove.launch.py` starts (`arm_commander:=false` to skip). `arm_commander` wraps MoveIt as
services Foxglove can call: `/arm_commander/plan` (GetMotionPlan with defaults filled in),
`/arm_commander/execute`, `/arm_commander/stop` and the latched status `/arm_commander/status`.

* Drag the ball's arrows: the translucent orange arm follows via `/compute_ik` (position only,
  `camera_link`); the ball turns red if the point is unreachable.
* Yaw / Shoulder / Wrist sliders and the *Preset* menu (read from `agrobot.srdf`) set the goal too.
  *Goal = current* resets it to where the arm is.
* *Plan* previews the trajectory on the ghost (*Replay* repeats it); *Execute* runs the last plan
  through MoveIt's `/execute_trajectory`; *Plan & Execute* does both; *Speed* is the velocity /
  acceleration scaling. The status line follows `/arm_commander/status`.
* *chassis* shows the rest of the robot faintly.

Studio on another machine: open a *Foxglove WebSocket* connection to `ws://<robot-ip>:8765`.
Keep camera bandwidth low on Wi-Fi: the Image panel uses `/camera/image_raw/compressed`.

`telemetry_monitor` publishes `/diagnostics` (System CPU/memory/temperature, a status per sensor
stream with measured vs expected rate, controller states, hardware components) and fixed-order plot
topics `/telemetry/wheel_velocity` [FL FR RL RR], `/telemetry/arm_position`, `/telemetry/heading_deg`,
`/telemetry/cpu_percent|mem_percent|cpu_temp_c`. Temperature thresholds: `temp_warn_c:=80`,
`temp_error_c:=90`.

Layouts are generated: edit `scripts/generate_layouts.py`, run it, and `--check` verifies every
message path against the ROS message definitions.

---

# RViz window

```bash
ros2 launch agrobot_telemetry telemetry.launch.py sim:=true   # with agrobot_gazebo sim.launch.py
ros2 launch agrobot_telemetry telemetry.launch.py             # with hardware.launch.py (run on a PC)
```


MoveIt's `move_group`; neither starts an RViz any more. On the real robot, use the same
`ROS_DOMAIN_ID` / DDS settings on the PC and the RPi.

## What the window shows (Displays panel groups)

| Group | Displays |
|---|---|
| Robot | RobotModel (arm included), TF (off), EKF odometry trail, wheel odometry (off), GPS and global-EKF odometry (off, for `mode:=gps`) |
| Sensors | LaserScan `/scan` (arm masked), raw `/scan_raw` (off), the arm camera `/camera/image_raw` (own image panel) |
| Navigation | `/map`, local costmap, global costmap (off), global + local plan, footprint, AMCL particles, AMCL pose (off) |
| Arm (MoveIt) | MotionPlanning display + panel: Plan & Execute, named poses, trajectory slider, **drag the camera marker** (Interact tool) |

Panels: Displays, **Navigation 2** (nav status, waypoint mode, cancel), Views, Tool Properties, Time.
Tools: Interact (needed for dragging the arm marker), Move Camera, 2D Pose Estimate (`/initialpose`),
2D Goal Pose (`/goal_pose`), Publish Point, Measure.

Fixed frame is `odom` (always present). Map-frame data appears once SLAM or AMCL publishes `map → odom`
(`autonomy.launch.py`).

## Tips

* Arm control: pick a named pose (`home`, `look_down`, `look_left`, ...) as the Goal State in the
  MotionPlanning panel → Plan & Execute, or drag the marker on the camera. IK is position-only
  (see `agrobot_moveit_config/README.md`).
* The MotionPlanning panel starts with Velocity/Accel. Scaling 0.10 (an RViz default that cannot be
  preset from launch); raise them to ~0.5 in the Planning tab for brisker arm moves. `move_group`
  itself defaults to 0.5, so requests that set no scaling (scripts, MoveGroup actions) are faster.
* Layout is yours to change: edit in RViz and *File → Save Config As*, then run with
  `rviz_config:=/path/to/my.rviz`.
* The IMU has no stock RViz display here (`rviz_imu_plugin` is not installed); orientation shows in the
  EKF odometry arrows and the robot model. GPS satellite views remain `mapviz_display.launch.py`.
