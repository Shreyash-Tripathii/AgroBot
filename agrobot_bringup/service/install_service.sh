#!/bin/bash
# Run ON THE HARDWARE RPi, once:   sudo ./install_service.sh [--user NAME] [--dry-run]
# Installs + enables agrobot-hardware.service (starts hardware.launch.py at every boot, restarts it
# if it crashes), /etc/default/agrobot (settings) and a sudoers rule so `agrobot_ctl.sh` can
# start/stop/restart it over ssh without a password.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
WS="$(cd "$DIR/../../../.." && pwd)"              # <ws>/src/AgroBot/agrobot_bringup/service -> <ws>
RUN_USER="${SUDO_USER:-$(id -un)}"; DRY=0
while [ $# -gt 0 ]; do case "$1" in
  --user) RUN_USER="$2"; shift 2;;
  --dry-run) DRY=1; shift;;
  *) echo "unknown arg $1"; exit 2;; esac; done
[ -f "$WS/install/setup.bash" ] || echo "WARNING: $WS/install/setup.bash missing: colcon build first"
HOME_DIR="$(getent passwd "$RUN_USER" | cut -d: -f6)"
UNIT="$(sed -e "s|@USER@|$RUN_USER|g" -e "s|@HOME@|$HOME_DIR|g" -e "s|@WS@|$WS|g" \
            -e "s|@RUN@|$DIR/agrobot_hardware_run.sh|g" "$DIR/agrobot-hardware.service.in")"
SUDOERS="$RUN_USER ALL=(root) NOPASSWD: /usr/bin/systemctl start agrobot-hardware, /usr/bin/systemctl stop agrobot-hardware, /usr/bin/systemctl restart agrobot-hardware, /usr/bin/systemctl reset-failed agrobot-hardware"
if [ $DRY = 1 ]; then echo "$UNIT"; echo "--- sudoers:"; echo "$SUDOERS"; exit 0; fi
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }
chmod +x "$DIR"/*.sh
echo "$UNIT" > /etc/systemd/system/agrobot-hardware.service
if [ ! -f /etc/default/agrobot ]; then cat > /etc/default/agrobot <<EOT
# Settings for agrobot-hardware.service and agrobot_ctl.sh (restart the service after editing).
AGROBOT_WS=$WS
# ROS_DOMAIN_ID=0
# RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CYCLONEDDS_URI=file:///home/$RUN_USER/cyclonedds.xml
# Extra hardware.launch.py arguments, e.g. "use_arm:=false mock_imu:=true"
HARDWARE_ARGS=""
EOT
echo "created /etc/default/agrobot: set ROS_DOMAIN_ID / CYCLONEDDS_URI there to match the rest of the robot"; fi
echo "$SUDOERS" > /etc/sudoers.d/agrobot-hardware; chmod 440 /etc/sudoers.d/agrobot-hardware
visudo -cf /etc/sudoers.d/agrobot-hardware >/dev/null || { rm -f /etc/sudoers.d/agrobot-hardware; echo "sudoers check failed"; exit 1; }
for g in dialout i2c gpio; do getent group $g >/dev/null && usermod -aG $g "$RUN_USER" || true; done
systemctl daemon-reload
systemctl enable agrobot-hardware.service
systemctl restart agrobot-hardware.service
echo "done. status:  systemctl status agrobot-hardware   logs:  journalctl -u agrobot-hardware -f"
