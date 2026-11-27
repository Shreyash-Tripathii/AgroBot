#!/usr/bin/env python3
"""
lidar_manager: starts / stops `ros2 launch agrobot_bringup laser.launch.py` on request and shows its output.

  /lidar_manager/start | stop | restart   std_srvs/Trigger
  /lidar_manager/state                    std_msgs/String   starting | running | stopped | crashed
  /lidar_manager/status                   std_msgs/String   same, with the reason: crashed: <why>
  /lidar_manager/log                      rcl_interfaces/Log  every line the launch prints (the terminal
                                          output of the traditional launch), latched history of the last 500;
                                          also echoed to this node's own stdout (journal)
  /diagnostics                            "LiDAR driver" (OK running, WARN starting/stopped, ERROR crashed)

Nothing here touches the serial port or the driver; the launch file does all of that. The launch is
"crashed" when it exits or when it reports `process has died`; it is then shut down so that start/restart
begin clean. There is no automatic restart: the Foxglove button is the restart.
"""
import os
import re
import signal
import subprocess
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')


def log_level(line):
    u = line.upper()
    if '[FATAL' in u or 'TERMINATE CALLED' in u or 'TRACEBACK' in u:
        return Log.FATAL
    if '[ERROR' in u or 'ERROR' in u or 'HAS DIED' in u:
        return Log.ERROR
    if '[WARN' in u or 'WARNING' in u:
        return Log.WARN
    if '[DEBUG' in u:
        return Log.DEBUG
    return Log.INFO


class LidarManager(Node):
    def __init__(self):
        super().__init__('lidar_manager')
        d = self.declare_parameter
        self.cmd = [c for c in d('command', ['ros2', 'launch', 'agrobot_bringup', 'laser.launch.py']).value if c]
        self.autostart = bool(d('autostart', True).value)
        self.ready_regex = re.compile(d('ready_regex', 'scan mode').value)
        self.died_regex = re.compile(d('died_regex', 'process has died').value)

        cb = ReentrantCallbackGroup()
        latched = QoSProfile(depth=500, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.log_pub = self.create_publisher(Log, '/lidar_manager/log', latched)
        self.status_pub = self.create_publisher(String, '/lidar_manager/status', 10)
        self.state_pub = self.create_publisher(String, '/lidar_manager/state', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        for name, fn in (('start', self._svc_start), ('stop', self._svc_stop), ('restart', self._svc_restart)):
            self.create_service(Trigger, f'/lidar_manager/{name}', fn, callback_group=cb)
        self.create_timer(1.0, self._tick, callback_group=cb)

        self.lock = threading.RLock()
        self.state, self.detail = 'stopped', ''
        self.proc, self.thread = None, None
        self.stop_requested = False
        self.started_at = None
        self.last_exit = None
        self.last_error = ''
        self._set('stopped')
        if self.autostart:
            self.start()

    # ------------------------------------------------------------------ helpers
    def _emit(self, name, line, level=None):
        m = Log()
        m.stamp = self.get_clock().now().to_msg()
        m.level = level if level is not None else log_level(line)
        m.name, m.msg = name, line
        m.file, m.function, m.line = '', '', 0
        self.log_pub.publish(m)

    def _note(self, text, level=Log.INFO):
        print(f'[lidar_manager] {text}', flush=True)
        self._emit('lidar_manager', text, level)

    def _set(self, state, detail=''):
        with self.lock:
            self.state, self.detail = state, detail
        self._publish_status()

    def _status_text(self):
        return self.state + (f': {self.detail}' if self.detail else '')

    def _publish_status(self):
        self.status_pub.publish(String(data=self._status_text()))
        self.state_pub.publish(String(data=self.state))

    def _tick(self):
        self._publish_status()
        st = DiagnosticStatus(name='LiDAR driver', hardware_id='lidar')
        st.level = {'running': DiagnosticStatus.OK, 'crashed': DiagnosticStatus.ERROR}.get(
            self.state, DiagnosticStatus.WARN)
        st.message = self._status_text()
        alive = self.proc is not None and self.proc.poll() is None
        up = (time.monotonic() - self.started_at) if (self.started_at and self.state == 'running') else 0.0
        for k, v in (('state', self.state), ('launch pid', self.proc.pid if alive else ''),
                     ('uptime_s', f'{up:.0f}'),
                     ('last exit code', '' if self.last_exit is None else self.last_exit),
                     ('last error line', self.last_error)):
            st.values.append(KeyValue(key=k, value=str(v)))
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(st)
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------------ process control
    def start(self):
        with self.lock:
            if self.thread and self.thread.is_alive():
                return False, f'already {self.state}'
            self.stop_requested = False
            self._set('starting')
            self.thread = threading.Thread(target=self._run, daemon=True)
            self.thread.start()
            return True, 'starting'

    def stop(self):
        with self.lock:
            t, p = self.thread, self.proc
            self.stop_requested = True
        if not (t and t.is_alive()):
            self._set('stopped')
            return True, 'was not running'
        self._note('stop requested')
        self._kill(signal.SIGINT)                     # Ctrl-C: the launch shuts its nodes down cleanly
        t.join(10.0)
        if t.is_alive():
            self._note('launch did not exit after SIGINT, sending SIGTERM', Log.WARN)
            self._kill(signal.SIGTERM)
            t.join(4.0)
        if t.is_alive():
            self._kill(signal.SIGKILL)
            t.join(3.0)
        self._set('stopped')
        return True, 'stopped'

    def _kill(self, sig):
        p = self.proc
        if p and p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), sig)
            except (ProcessLookupError, PermissionError):
                pass

    def _run(self):
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_LOGGING_BUFFERED_STREAM='0',
                   RCUTILS_COLORIZED_OUTPUT='0')
        self._note('launching: ' + ' '.join(self.cmd))
        try:
            self.proc = subprocess.Popen(self.cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                         bufsize=1, start_new_session=True, errors='replace', env=env)
        except OSError as e:
            self.last_error = str(e)
            self._set('crashed', f'cannot start: {e}')
            return
        t0 = self.started_at = time.monotonic()
        died = False
        for raw in self.proc.stdout:
            line = ANSI.sub('', raw.rstrip())
            if not line:
                continue
            print(line, flush=True)                   # journal: same as the terminal
            lvl = log_level(line)
            self._emit('laser.launch', line, lvl)
            if lvl >= Log.ERROR:
                self.last_error = line[:160]
            if self.state != 'running' and self.ready_regex.search(line):
                self._set('running')
            if self.died_regex.search(line) and not died and not self.stop_requested:
                died = True                           # driver died: end the launch so a relaunch starts clean
                self._set('crashed', 'sllidar_node died (see log)')
                threading.Timer(1.0, self._kill, args=(signal.SIGINT,)).start()
        rc = self.proc.wait()
        self.last_exit = rc
        if self.stop_requested:
            self._note(f'launch stopped (exit code {rc})')
            return
        if not died:
            self._set('crashed', f'launch exited with code {rc} after {time.monotonic() - t0:.0f} s')
        self._note(f'launch ended: {self._status_text()}', Log.ERROR)

    # ------------------------------------------------------------------ services
    def _svc_start(self, req, res):
        res.success, res.message = self.start()
        return res

    def _svc_stop(self, req, res):
        res.success, res.message = self.stop()
        return res

    def _svc_restart(self, req, res):
        self.stop()
        res.success, res.message = self.start()
        return res

    def shutdown(self):
        self.stop_requested = True
        self._kill(signal.SIGINT)
        if self.thread:
            self.thread.join(8.0)
        self._kill(signal.SIGKILL)


def main():
    rclpy.init()
    node = LidarManager()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    node.shutdown()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
