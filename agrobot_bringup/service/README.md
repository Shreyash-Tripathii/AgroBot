# Run hardware.launch.py at boot (hardware RPi)

On the hardware RPi, once, after `colcon build`:

```bash
cd <ws>/src/AgroBot/agrobot_bringup/service
sudo ./install_service.sh            # installs, enables and starts agrobot-hardware.service
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
