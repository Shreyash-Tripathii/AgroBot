# AgroBot Arm Planner (Foxglove extension)

A MoveIt-style planning panel for the 3-DOF camera arm: the arm model, a draggable ball end effector,
your SRDF preset poses, and Plan / Execute through MoveIt.

Needs on the robot side (started by `ros2 launch agrobot_telemetry foxglove.launch.py`):
`move_group`, `/robot_description`, `/robot_description_semantic`, `/compute_ik`, and
`arm_commander` (`/arm_commander/{plan,execute,stop,status}`).

## Build and install

```bash
sudo apt install nodejs npm        # once
cd agrobot_telemetry/foxglove/extension
npm install
npm run local-install              # installs into ~/.foxglove-studio/extensions, then restart Studio
npm run package                    # or: build a .foxe to install on another machine
```

## Using it

- Orange ball = goal end effector. Drag its arrows: the translucent orange arm follows (IK, position
  only). The ball turns red when the point is unreachable.
- Sliders and the Preset menu also set the goal. Presets come from `agrobot.srdf`.
- Plan previews the trajectory on the ghost, Execute runs the last plan through MoveIt,
  Plan & Execute does both, Stop cancels.
