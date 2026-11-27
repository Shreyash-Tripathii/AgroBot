#!/bin/bash
# Run on your PC:   ./agrobot_ctl.sh <user@host> [status|restart|start|stop|logs]
# or set AGROBOT_HOST=user@host once.  Default command: status.
#   status   service state, uptime, last log lines, and the ros2_control controllers
#   restart  restart hardware.launch.py (use after a crash or after editing /etc/default/agrobot)
#   logs     follow the service log (Ctrl-C to leave)
if [ $# -ge 1 ] && [[ "$1" == *@* || "$1" == *.* ]]; then HOST="$1"; shift; else HOST="${AGROBOT_HOST:-}"; fi
CMD="${1:-status}"
[ -n "$HOST" ] || { echo "usage: $0 <user@host> [status|restart|start|stop|logs]   (or export AGROBOT_HOST)"; exit 2; }
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=6 "$HOST")
case "$CMD" in
  status)
    "${SSH[@]}" 'systemctl is-enabled agrobot-hardware; systemctl status agrobot-hardware --no-pager -n 12 | cut -c1-160;
      echo; echo "== controllers"; . /opt/ros/jazzy/setup.bash; WS=$(sed -n "s/^AGROBOT_WS=//p" /etc/default/agrobot);
      . /etc/default/agrobot 2>/dev/null; export ROS_DOMAIN_ID RMW_IMPLEMENTATION CYCLONEDDS_URI; . "$WS/install/setup.bash";
      timeout 10 ros2 control list_controllers 2>&1 | head -12' ;;
  restart) "${SSH[@]}" 'sudo -n systemctl reset-failed agrobot-hardware; sudo -n systemctl restart agrobot-hardware' && echo "restarted"; sleep 3; exec "$0" "$HOST" status ;;
  start|stop) "${SSH[@]}" "sudo -n systemctl $CMD agrobot-hardware" && echo "$CMD ok" ;;
  logs) exec "${SSH[@]}" -t 'journalctl -u agrobot-hardware -f -n 50 --no-pager' ;;
  *) echo "unknown command $CMD"; exit 2 ;;
esac
