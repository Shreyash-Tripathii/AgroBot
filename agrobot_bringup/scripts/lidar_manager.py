#!/usr/bin/env python3
"""
lidar_manager: owns the sllidar_node process so it can be started, stopped and relaunched at runtime
(from Foxglove, the CLI, anything that can call a service) and so its output is never lost.

  /lidar_manager/start | stop | restart   std_srvs/Trigger
  /lidar_manager/state                    std_msgs/String   starting | running | stopped | crashed
  /lidar_manager/status                   std_msgs/String   same, with the reason: crashed: <why>
  /lidar_manager/log                      rcl_interfaces/Log  the driver's stdout/stderr, line by line,
                                          INCLUDING what a crashing process prints after /rosout is gone
  /diagnostics                            "LiDAR driver" (OK running, WARN starting/stopped, ERROR crashed)

The driver is started through lidar_wake.py (waits until the lidar answers with its motor on, see there).
If it dies it is restarted automatically up to `max_auto_restarts` times in a row; after that it stays
"crashed" until someone presses start/restart. `stop` is final: no automatic restart until start.
"""
import glob
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from builtin_interfaces.msg import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import lidar_wake  # noqa: E402

ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
# "[INFO] [1795.12] [sllidar_node]: text" -> "text" (level and name are separate log fields here)
PREFIX = re.compile(r'^\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*(?:\[[\d.]+\]\s*)?(?:\[[^\]]+\]:\s*)?')


def log_level(line):
    u = line.upper()
    if '[FATAL' in u or 'TERMINATE CALLED' in u or 'TRACEBACK' in u:
        return Log.FATAL
    if '[ERROR' in u or 'ERROR' in u:
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
        self.port_param = d('serial_port', 'auto').value
        self.baud = int(d('serial_baudrate', 115200).value)
        self.frame_id = d('frame_id', 'lidar_link').value
        self.inverted = bool(d('inverted', False).value)
        self.scan_topic = d('scan_topic', 'scan_raw').value
        self.autostart = bool(d('autostart', True).value)
        self.auto_restart = bool(d('auto_restart', True).value)
        self.max_auto = int(d('max_auto_restarts', 5).value)
        self.restart_delay = float(d('restart_delay', 3.0).value)
        self.use_wake = bool(d('wake', True).value)
        self.wake_timeout = float(d('wake_timeout', 90.0).value)
        self.ready_regex = re.compile(d('ready_regex', 'scan mode').value)
        self.driver_cmd = list(d('driver_cmd', ['']).value)          # test override; '' = real sllidar_node
        self.driver_cmd = [c for c in self.driver_cmd if c]

        cb = ReentrantCallbackGroup()
        self.log_pub = self.create_publisher(Log, '/lidar_manager/log', 200)
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
        self.port = None
        self.started_at = None
        self.restarts = 0
        self.last_exit = None
        self.last_error = ''
        self._set('stopped')
        if self.autostart:
            self.start()

    # ------------------------------------------------------------------ helpers
    def _emit(self, name, text, level=None):
        for line in text.splitlines() or ['']:
            m = Log()
            m.stamp = self.get_clock().now().to_msg()
            m.level = level if level is not None else log_level(line)
            m.name, m.msg = name, line
            m.file, m.function, m.line = '', '', 0
            self.log_pub.publish(m)

    def _note(self, text, level=Log.INFO):
        self.get_logger().info(text) if level < Log.WARN else self.get_logger().warning(text)
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
        up = (time.monotonic() - self.started_at) if (self.started_at and self.state == 'running') else 0.0
        for k, v in (('state', self.state), ('port', self.port or ''), ('pid', self.proc.pid if self.proc and self.proc.poll() is None else ''),
                     ('uptime_s', f'{up:.0f}'), ('automatic restarts', self.restarts),
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
            self.restarts = 0
            self._set('starting')
            self.thread = threading.Thread(target=self._supervise, daemon=True)
            self.thread.start()
            return True, 'starting'

    def stop(self):
        with self.lock:
            t = self.thread
            self.stop_requested = True
            p = self.proc
        if not (t and t.is_alive()) and not (p and p.poll() is None):
            self._set('stopped')
            return True, 'was not running'
        self._note('stop requested')
        self._signal(signal.SIGINT)
        t.join(8.0)
        if t.is_alive():
            self._note('driver ignored SIGINT, sending SIGKILL', Log.WARN)
            self._signal(signal.SIGKILL)
            t.join(3.0)
        if self.use_wake and self.port and not self.driver_cmd:
            lidar_wake.motor_off(self.port, self.baud)
        self._set('stopped')
        return True, 'stopped'

    def _signal(self, sig):
        p = self.proc
        if p and p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), sig)
            except (ProcessLookupError, PermissionError):
                pass

    def _build_cmd(self):
        if self.driver_cmd:
            return self.driver_cmd
        node = os.path.join(get_package_prefix('sllidar_ros2'), 'lib', 'sllidar_ros2', 'sllidar_node')
        cmd = [node, '--ros-args', '-r', f'scan:={self.scan_topic}',
               '-p', 'channel_type:=serial', '-p', f'serial_port:={self.port}',
               '-p', f'serial_baudrate:={self.baud}', '-p', f'frame_id:={self.frame_id}',
               '-p', f'inverted:={str(self.inverted).lower()}', '-p', 'angle_compensate:=true']
        if self.use_wake:
            cmd = [sys.executable, os.path.join(os.path.dirname(os.path.realpath(__file__)), 'lidar_wake.py'),
                   '--port', self.port, '--baud', str(self.baud), '--timeout', str(self.wake_timeout), '--'] + cmd
        return cmd

    def _supervise(self):
        consecutive = 0
        while not self.stop_requested:
            if not self.driver_cmd:
                if self.port_param == 'auto':
                    self._set('starting', 'looking for the lidar')
                    self._note('probing /dev/ttyUSB* for the RPLidar ...')
                    self.port = lidar_wake.detect_port(self.baud, 20.0)
                else:
                    self.port = self.port_param
                if not self.port:
                    self._fail('RPLidar not found on any /dev/ttyUSB* (check power / cable)', None)
                    if self._retry(consecutive := consecutive + 1):
                        continue
                    return
            cmd = self._build_cmd()
            self._note('launching: ' + ' '.join(cmd[-8:] if len(cmd) > 8 else cmd))
            self._set('starting', 'waiting for the lidar to answer' if self.use_wake else '')
            try:
                self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                             bufsize=1, start_new_session=True, errors='replace')
            except OSError as e:
                self._fail(f'cannot start driver: {e}', None)
                return
            t0 = time.monotonic()
            self.started_at = t0
            for raw in self.proc.stdout:
                line = ANSI.sub('', raw.rstrip())
                if not line:
                    continue
                self._emit('sllidar_node', PREFIX.sub('', line) or line, log_level(line))
                if log_level(line) >= Log.ERROR:
                    self.last_error = line[:160]
                if self.state != 'running' and self.ready_regex.search(line):
                    self._set('running')
            rc = self.proc.wait()
            self.last_exit = rc
            if self.stop_requested:
                self._note(f'driver stopped (exit code {rc})')
                return
            self._fail(f'driver exited with code {rc} after {time.monotonic() - t0:.0f} s', rc)
            if time.monotonic() - t0 > 60:
                consecutive = 0
            consecutive += 1
            if not self._retry(consecutive):
                return

    def _fail(self, why, rc):
        self._note(why, Log.ERROR)
        self._set('crashed', why)

    def _retry(self, consecutive):
        if not self.auto_restart or self.stop_requested:
            return False
        if consecutive > self.max_auto:
            self._note(f'giving up after {self.max_auto} automatic restarts: press restart', Log.ERROR)
            return False
        self.restarts += 1
        self._note(f'automatic restart {consecutive}/{self.max_auto} in {self.restart_delay:.0f} s', Log.WARN)
        end = time.monotonic() + self.restart_delay
        while time.monotonic() < end and not self.stop_requested:
            time.sleep(0.1)
        return not self.stop_requested

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
        try:
            self.stop()
        except Exception:                      # context already shut down by Ctrl-C: just make sure it's dead
            self._signal(signal.SIGKILL)


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
