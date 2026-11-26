# agrobot_gazebo — Gazebo (gz sim) simulation of AgroBot

One command runs the **same ros2_control stack, topics and frames as the real robot** in a
crop-field world, so Nav2, SLAM, the EKF, MoveIt and camera scripts cannot tell the difference.

```bash
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-joint-state-publisher-gui   # once
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash

ros2 launch agrobot_gazebo sim.launch.py                         # field + Gazebo window
ros2 launch agrobot_gazebo sim.launch.py gui:=false              # headless
ros2 launch agrobot_gazebo sim.launch.py world:=empty_test use_arm:=false

ros2 launch agrobot_telemetry telemetry.launch.py sim:=true      # the ONE RViz window (own terminal)
```

The sim launch starts **no RViz**. Visualisation is the separate `agrobot_telemetry` package (map,
costmaps, lidar, arm camera, arm model, Nav2 panel and MoveIt motion planning in one window).

Drive it like the real robot (plain `Twist` on `/cmd_vel`, through the heading-hold controller):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## What is simulated

| Real robot | Simulation |
|---|---|
| ESP32 drive + arm (`ESP32SystemInterface`) | `gz_ros2_control/GazeboSimSystem` (same joints and interfaces) |
| BNO085 (`imu_game`, `imu_mag`) | gz IMU sensors with the same names → same ros2_control state interfaces → same broadcasters (`/imu/data_game` 50 Hz, `/imu/data_mag` 5 Hz) |
| yaw → fuzzy scheduler → diff drive | yaw → diff drive (`config/controllers_sim.yaml`; the scheduler tunes the ESP32's motor PID, which Gazebo does not have) |
| RPLidar + arm mask | gz `gpu_lidar` → `/scan_raw` → the **same** `laser_filter.yaml` chain → `/scan` |
| u-blox GPS | gz `navsat` → `/gps/fix` |
| (arm-tip camera) | gz RGB camera 640×480 @ 15 Hz → `/camera/image_raw`, `/camera/camera_info`, frame `camera_optical_frame` |
| EKF | `agrobot_bringup/config/ekf.yaml`, `use_sim_time` |

Everything runs on `/clock`. Launch arguments: `world`, `use_arm`, `use_ekf`, `moveit`, `gui`, `x y z yaw`.

## Navigation in simulation

```bash
# terminal 1: ros2 launch agrobot_gazebo sim.launch.py
ros2 launch agrobot_navigation autonomy.launch.py mode:=mapping sim:=true             # SLAM
ros2 launch agrobot_navigation autonomy.launch.py explore:=true sim:=true             # autonomous
ros2 launch agrobot_navigation autonomy.launch.py mode:=localization sim:=true        # AMCL
ros2 launch agrobot_navigation autonomy.launch.py mode:=gps sim:=true                 # GPS + navsat
```

`sim:=true` keeps maps in `agrobot_navigation/maps/sim/` (the real robot's maps are untouched),
skips the u-blox driver and applies `agrobot_navigation/config/sim_overrides.yaml`.

## MoveIt (arm planning)

move_group (`agrobot_moveit_config`) starts with the sim by default (`moveit:=false` to skip) and
plans against the simulated `arm_controller`. Plan, execute and drag the camera marker from the
MotionPlanning display in the telemetry window; read the camera on `/camera/image_raw`.
Details: `agrobot_moveit_config/README.md`.

## World

`worlds/agro_field.sdf` is generated (seeded, reproducible):

```bash
python3 scripts/generate_field_world.py --out worlds/agro_field.sdf --seed 7 \
    --rows 6 --plants 12 --row-spacing 0.75 --plant-spacing 0.6 --lat 26.8467 --lon 80.9462
```

Crop rows run along +x from x = 2 m; the robot spawns in the headland at the origin. Each plant
model carries a `gz-sim-label-system` label (1 cabbage, 2 tomato plant, 3 carrot) so a gz
segmentation / bounding-box camera can later provide ground truth for an OpenCV detector.

## Extending

* **Custom Nav2 planner/controller plugins:** register them in
  `agrobot_navigation/config/nav2_params.yaml` (`planner_plugins` / `controller_plugins`); the
  simulation picks them up unchanged. Run `autonomy.launch.py ... sim:=true`.
* **OpenCV vegetable detector:** subscribe to `/camera/image_raw` (`sensor_msgs/Image`, `rgb8`) with
  `use_sim_time`. Use `cv_bridge`. Move the camera with `/arm_controller/joint_trajectory`.
  Suggested camera poses (yaw, shoulder, wrist): home `0 0 0` (forward, horizon), look down at the
  crop `0 0 0.55`. Do **not** use a large shoulder+wrist sum (e.g. `0.5 0.9`): the camera then
  stares at the robot's own chassis.
* **Other worlds:** `world:=/path/to/world.sdf` (needs the Physics/Sensors/Imu/NavSat systems and
  `<spherical_coordinates>`, see `worlds/empty_test.sdf`).

## Notes

* The arm position controller in gz_ros2_control uses a per-step P gain
  (`position_gain` in `agrobot.ros2_control.xacro`); it must stay below 1.
* Rendering uses ogre2. Without a usable GPU, run `gui:=false` (headless EGL) or set
  `LIBGL_ALWAYS_SOFTWARE=1`.
