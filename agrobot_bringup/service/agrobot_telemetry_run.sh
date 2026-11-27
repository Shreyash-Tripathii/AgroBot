#!/bin/bash
# ExecStart of agrobot-telemetry.service (2nd RPi): environment + telemetry.launch.py WITHOUT autonomy
# (run autonomy.launch.py yourself, separately). Extra arguments come from TELEMETRY_ARGS.
. "$(dirname "$(readlink -f "$0")")/agrobot_env.sh"
echo "[agrobot-telemetry] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default} args='${TELEMETRY_ARGS}'"
# shellcheck disable=SC2086
exec ros2 launch agrobot_bringup telemetry.launch.py ${TELEMETRY_ARGS} autonomy:=false
