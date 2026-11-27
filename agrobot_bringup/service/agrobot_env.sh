#!/bin/bash
# Sourced (not executed) by the service and by agrobot_ctl.sh: the same ROS environment everywhere.
# Edit /etc/default/agrobot (ROS_DOMAIN_ID, RMW_IMPLEMENTATION, CYCLONEDDS_URI, HARDWARE_ARGS ...) to
# change it; no rebuild or reinstall needed, just `agrobot_ctl.sh <host> restart`.
set +u
[ -f /etc/default/agrobot ] && { set -a; . /etc/default/agrobot; set +a; }
. "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
[ -f "${AGROBOT_WS:?AGROBOT_WS not set (see /etc/default/agrobot)}/install/setup.bash" ] \
  && . "$AGROBOT_WS/install/setup.bash"
export PYTHONUNBUFFERED=1
