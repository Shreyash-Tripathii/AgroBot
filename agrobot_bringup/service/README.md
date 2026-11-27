# Run the launch files at boot (both RPis)

On the hardware RPi, once, after `colcon build`:

```bash
cd <ws>/src/AgroBot/agrobot_bringup/service
sudo ./install_service.sh --role hardware    # installs, enables and starts agrobot-hardware.service
sudo nano /etc/default/agrobot       # set ROS_DOMAIN_ID / RMW_IMPLEMENTATION / CYCLONEDDS_URI to match your
                                     # other RPi; HARDWARE_ARGS="use_arm:=false" etc. Then restart (below).
```

From then on it starts at every power-on and is restarted 3 s after any crash.

On the RPi: `systemctl status agrobot-hardware`, `journalctl -u agrobot-hardware -f`.

From your PC (needs ssh key access; `AGROBOT_HOST=user@host` can replace the first argument):

```bash
./agrobot_ctl.sh user@host status     # service state, last logs, ros2_control controllers
./agrobot_ctl.sh user@host restart    # also: start | stop | logs
```

## 2nd RPi (lidar, Foxglove bridge): telemetry.launch.py, no autonomy

```bash
sudo ./install_service.sh --role telemetry   # agrobot-telemetry.service = telemetry.launch.py autonomy:=false
```

Same settings file (`TELEMETRY_ARGS` for extra launch arguments). Autonomy is deliberately not part
of it: start `autonomy.launch.py` yourself (or via `telemetry.launch.py autonomy:=true mode:=...` by hand).
Use `--role telemetry` with the control script: `./agrobot_ctl.sh --role telemetry user@host status|restart|logs`.
The lidar driver itself is restarted from Foxglove (Engineering layout: Relaunch / Stop / Start).
