#!/bin/bash
# ExecStart of agrobot-hardware.service: environment + hardware.launch.py (args from HARDWARE_ARGS).
. "$(dirname "$(readlink -f "$0")")/agrobot_env.sh"
echo "[agrobot-hardware] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default} args='${HARDWARE_ARGS}'"
# shellcheck disable=SC2086
exec ros2 launch agrobot_bringup hardware.launch.py ${HARDWARE_ARGS}
