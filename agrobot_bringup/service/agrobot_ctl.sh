#!/bin/bash
# Run on your PC:   ./agrobot_ctl.sh [--role hardware|telemetry] <user@host> [status|restart|start|stop|logs]
# (AGROBOT_HOST=user@host replaces the host argument; default role hardware, default command status)
#   status   service state, last log lines, then ros2_control controllers (hardware) or lidar state (telemetry)
#   restart  restart the whole launch (after a crash or after editing /etc/default/agrobot)
#   logs     follow the service log (Ctrl-C to leave)
ROLE=hardware; [ "$1" = "--role" ] && { ROLE="$2"; shift 2; }
if [ $# -ge 1 ] && [[ "$1" == *@* || "$1" == *.* ]]; then HOST="$1"; shift; else HOST="${AGROBOT_HOST:-}"; fi
CMD="${1:-status}"; SVC="agrobot-$ROLE"
[ -n "$HOST" ] || { echo "usage: $0 [--role hardware|telemetry] <user@host> [status|restart|start|stop|logs]"; exit 2; }
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=6 "$HOST")
ENV='. /opt/ros/jazzy/setup.bash; WS=$(sed -n "s/^AGROBOT_WS=//p" /etc/default/agrobot); . /etc/default/agrobot 2>/dev/null;
     export ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI; . "$WS/install/setup.bash"'
case "$CMD" in
  status)
    if [ "$ROLE" = hardware ]; then EXTRA='echo "== controllers"; timeout 10 ros2 control list_controllers 2>&1 | head -12'
    else EXTRA='echo "== lidar"; timeout 10 ros2 topic echo /lidar_manager/status --once --field data 2>&1 | head -2'; fi
    "${SSH[@]}" "systemctl is-enabled $SVC; systemctl status $SVC --no-pager -n 12 | cut -c1-160; echo; $ENV; $EXTRA" ;;
  restart) "${SSH[@]}" "sudo -n systemctl reset-failed $SVC; sudo -n systemctl restart $SVC" && echo "restarted"; sleep 3; exec "$0" --role "$ROLE" "$HOST" status ;;
  start|stop) "${SSH[@]}" "sudo -n systemctl $CMD $SVC" && echo "$CMD ok" ;;
  logs) exec "${SSH[@]}" -t "journalctl -u $SVC -f -n 50 --no-pager" ;;
  *) echo "unknown command $CMD"; exit 2 ;;
esac
