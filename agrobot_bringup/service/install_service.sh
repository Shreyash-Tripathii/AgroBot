#!/bin/bash
# Run ON THE RPi, once:   sudo ./install_service.sh --role hardware|telemetry [--user NAME] [--dry-run]
#   hardware   (the RPi wired to the ESP32)  -> agrobot-hardware.service  = hardware.launch.py
#   telemetry  (the 2nd RPi with the lidar)  -> agrobot-telemetry.service = telemetry.launch.py autonomy:=false
# Installs + enables the service (starts at every boot, restarted if it crashes), creates
# /etc/default/agrobot (settings) and a sudoers rule so `agrobot_ctl.sh` can start/stop/restart it over ssh.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$DIR/../../../.." && pwd)"              # <ws>/src/AgroBot/agrobot_bringup/service -> <ws>
RUN_USER="${SUDO_USER:-$(id -un)}"; DRY=0; ROLE=hardware
while [ $# -gt 0 ]; do case "$1" in
  --user) RUN_USER="$2"; shift 2;;
  --role) ROLE="$2"; shift 2;;
  --dry-run) DRY=1; shift;;
  *) echo "unknown arg $1"; exit 2;; esac; done
case "$ROLE" in
  hardware)  DESC="AgroBot hardware (ros2_control, ESP32, IMU, controllers, EKF, MoveIt)";;
  telemetry) DESC="AgroBot telemetry (Foxglove bridge, lidar manager, arm planner services)";;
  *) echo "--role must be hardware or telemetry"; exit 2;; esac
SVC="agrobot-$ROLE"
[ -f "$WS/install/setup.bash" ] || echo "WARNING: $WS/install/setup.bash missing: colcon build first"
HOME_DIR="$(getent passwd "$RUN_USER" | cut -d: -f6)"
UNIT="$(sed -e "s|@DESC@|$DESC|g" -e "s|@USER@|$RUN_USER|g" -e "s|@HOME@|$HOME_DIR|g" -e "s|@WS@|$WS|g" \
            -e "s|@RUN@|$DIR/agrobot_${ROLE}_run.sh|g" "$DIR/agrobot.service.in")"
CTL="/usr/bin/systemctl"
SUDOERS="$RUN_USER ALL=(root) NOPASSWD: $CTL start $SVC, $CTL stop $SVC, $CTL restart $SVC, $CTL reset-failed $SVC"
if [ $DRY = 1 ]; then echo "$UNIT"; echo "--- sudoers:"; echo "$SUDOERS"; exit 0; fi
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
chmod +x "$DIR"/*.sh
echo "$UNIT" > "/etc/systemd/system/$SVC.service"
if [ ! -f /etc/default/agrobot ]; then cat > /etc/default/agrobot <<EOT
# Settings for the agrobot-* services and agrobot_ctl.sh (restart the service after editing).
AGROBOT_WS=$WS
# ROS_DOMAIN_ID=0
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CYCLONEDDS_URI=file:///home/$RUN_USER/cyclonedds.xml
# Extra hardware.launch.py arguments (hardware RPi), e.g. "use_arm:=false mock_imu:=true"
HARDWARE_ARGS=""
# Extra telemetry.launch.py arguments (2nd RPi), e.g. "lidar_auto_restart:=false filter_arm:=false"
TELEMETRY_ARGS=""
EOT
echo "created /etc/default/agrobot: set ROS_DOMAIN_ID / CYCLONEDDS_URI there to match the rest of the robot"; fi
echo "$SUDOERS" > "/etc/sudoers.d/$SVC"; chmod 440 "/etc/sudoers.d/$SVC"
visudo -cf "/etc/sudoers.d/$SVC" >/dev/null || { rm -f "/etc/sudoers.d/$SVC"; echo "sudoers check failed"; exit 1; }
for g in dialout i2c gpio; do getent group $g >/dev/null && usermod -aG $g "$RUN_USER" || true; done
systemctl daemon-reload
systemctl enable "$SVC.service"
systemctl restart "$SVC.service"
echo "done. status:  systemctl status $SVC   logs:  journalctl -u $SVC -f"
