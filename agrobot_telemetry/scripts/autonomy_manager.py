#!/usr/bin/env python3
"""
autonomy_manager: start, switch and shut down agrobot_navigation's autonomy.launch.py from a service call
(Foxglove buttons), and report what is going on inside it.

  /autonomy_manager/start_explore | start_mapping | start_localization | start_gps | stop   std_srvs/Trigger
        a start_* while another mode runs first shuts that one down (mapping saves its map on the way)
  /autonomy_manager/state    std_msgs/String   stopped | starting | running | stopping | crashed
  /autonomy_manager/mode     std_msgs/String   explore | mapping | localization | gps | none
  /autonomy_manager/log      rcl_interfaces/Log  everything the launch prints (transient_local, 300 lines)
  /diagnostics               hardware_id "nav_launch"     "Autonomy launch" + one entry per ROS node it started
                             hardware_id "nav_lifecycle"  lifecycle state of every lifecycle node in it

Parameters: sim (bool, passes sim:=true), map_name (used by mapping to save / localization to load),
extra_args (string, e.g. "use_emra:=true"), stop_timeout (s, graceful shutdown before SIGTERM / SIGKILL).
"""
import os
import re
import shlex
import signal
import subprocess
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Log
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
PROC_TAG = re.compile(r'^\[([\w.\-]+?)(?:-\d+)?\]\s*')       # "[controller_server-3] ..." -> controller_server
LEVEL_TAG = re.compile(r'^\[(DEBUG|INFO|WARN|ERROR|FATAL)\]\s*(?:\[[\d.]+\]\s*)?(?:\[[^\]]+\]:\s*)?')
IGNORE_NODES = re.compile(r'(^/?_)|ros2cli|launch_ros|transform_listener_impl|rviz|foxglove|autonomy_manager|arm_commander|'
                          r'lidar_manager|telemetry_monitor|daemon')
MODES = ('explore', 'mapping', 'localization', 'gps')
STATE_LEVEL = {'active': DiagnosticStatus.OK, 'inactive': DiagnosticStatus.WARN,
               'unconfigured': DiagnosticStatus.WARN, 'finalized': DiagnosticStatus.ERROR}


def log_level(line):
    u = line.upper()
    if '[FATAL' in u or 'TRACEBACK' in u or 'TERMINATE CALLED' in u:
        return Log.FATAL
    if '[ERROR' in u:
        return Log.ERROR
    if '[WARN' in u:
        return Log.WARN
    if '[DEBUG' in u:
        return Log.DEBUG
    return Log.INFO


class AutonomyManager(Node):
    def __init__(self):
        super().__init__('autonomy_manager')
        d = self.declare_parameter
        self.sim = bool(d('sim', False).value)
        self.stop_timeout = float(d('stop_timeout', 30.0).value)
        d('map_name', '')
        d('extra_args', '')

        cb = ReentrantCallbackGroup()
        self.cb = cb
        self.log_pub = self.create_publisher(Log, '/autonomy_manager/log', QoSProfile(
            depth=300, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/autonomy_manager/state', latched)
        self.mode_pub = self.create_publisher(String, '/autonomy_manager/mode', latched)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        for m in MODES:
            self.create_service(Trigger, f'/autonomy_manager/start_{m}',
                                lambda req, res, m=m: self._svc_start(m, res), callback_group=cb)
        self.create_service(Trigger, '/autonomy_manager/stop', self._svc_stop, callback_group=cb)
        self.create_timer(1.0, self._tick, callback_group=cb)

        self.lock = threading.RLock()
        self.state, self.detail, self.mode = 'stopped', '', 'none'
        self.proc = None
        self.thread = None
        self.stop_requested = False
        self.started_at = None
        self.baseline = set()
        self.seen = {}                 # fqn -> last time it was present
        self.lc_clients = {}              # fqn -> GetState client
        self.pending = {}              # fqn -> (future, t_sent)
        self.lifecycle = {}            # fqn -> (label, t)
        self.last_error = ''
        self.last_exit = None
        self._publish_state()

    # ------------------------------------------------------------------ small helpers
    def _emit(self, name, text, level=None):
        for line in text.splitlines() or ['']:
            m = Log()
            m.stamp = self.get_clock().now().to_msg()
            m.level = level if level is not None else log_level(line)
            m.name, m.msg = name, line
            m.file, m.function, m.line = '', '', 0
            self.log_pub.publish(m)
            if name != 'autonomy_manager':
                print(f'[{name}] {line}', flush=True)

    def _note(self, text, level=Log.INFO):
        (self.get_logger().warning if level >= Log.WARN else self.get_logger().info)(text)
        self._emit('autonomy_manager', text, level)

    def _set(self, state, detail='', mode=None):
        with self.lock:
            self.state, self.detail = state, detail
            if mode is not None:
                self.mode = mode
        self._publish_state()

    def _publish_state(self):
        self.state_pub.publish(String(data=self.state))
        self.mode_pub.publish(String(data=self.mode))

    def _nodes_now(self):
        out = set()
        for name, ns in self.get_node_names_and_namespaces():
            fqn = (ns.rstrip('/') + '/' + name)
            if not IGNORE_NODES.search(fqn):
                out.add(fqn)
        return out

    # ------------------------------------------------------------------ process control
    def start(self, mode):
        if self.state in ('starting', 'running', 'stopping'):
            self._note(f'switching {self.mode} -> {mode}: shutting the current launch down first')
            self.stop()
        with self.lock:
            self.stop_requested = False
            self.baseline = self._nodes_now()
            self.seen.clear()
            self.lifecycle.clear()
            self.lc_clients.clear()
            self.pending.clear()
            self.last_error, self.last_exit = '', None
            args = ['ros2', 'launch', 'agrobot_navigation', 'autonomy.launch.py',
                    f'mode:={"mapping" if mode == "explore" else mode}',
                    f'explore:={"true" if mode == "explore" else "false"}']
            if self.sim:
                args.append('sim:=true')
            map_name = self.get_parameter('map_name').value
            if map_name:
                args.append(f'map_name:={map_name}')
            args += shlex.split(self.get_parameter('extra_args').value or '')
            self._note('starting: ' + ' '.join(args))
            try:
                self.proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                             bufsize=1, start_new_session=True, errors='replace')
            except OSError as e:
                self._set('crashed', f'cannot start: {e}', mode)
                return False, f'cannot start: {e}'
            self.started_at = time.monotonic()
            self._set('starting', '', mode)
            self.thread = threading.Thread(target=self._reader, args=(self.proc, mode), daemon=True)
            self.thread.start()
        return True, f'starting {mode}'

    def _reader(self, proc, mode):
        for raw in proc.stdout:
            line = ANSI.sub('', raw.rstrip())
            if not line:
                continue
            level = log_level(line)
            tag = PROC_TAG.match(line)
            name = tag.group(1) if tag else 'autonomy'
            body = LEVEL_TAG.sub('', PROC_TAG.sub('', line, 1)) or line
            self._emit(name, body, level)
            if level >= Log.ERROR:
                self.last_error = f'{name}: {body}'[:200]
        rc = proc.wait()
        self.last_exit = rc
        if self.stop_requested:
            self._note(f'autonomy launch stopped (exit code {rc})')
            self._set('stopped', '', 'none')
        elif rc == 0:
            self._note('autonomy launch finished (exit code 0)')
            self._set('stopped', 'finished', 'none')
        else:
            self._note(f'autonomy launch died with exit code {rc}', Log.ERROR)
            self._set('crashed', f'exit code {rc}')
        self._mark_stopped()

    def _mark_stopped(self):
        """One last 'stopped' (STALE) status for every node so the Foxglove panels do not show old data."""
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for fqn in sorted(self.seen):
            for hw in ('nav_launch', 'nav_lifecycle'):
                if hw == 'nav_lifecycle' and fqn not in self.lc_clients:
                    continue
                arr.status.append(DiagnosticStatus(level=DiagnosticStatus.STALE, name=fqn, hardware_id=hw,
                                                   message='stopped'))
        if arr.status:
            self.diag_pub.publish(arr)
        self.seen.clear()
        self.lc_clients.clear()
        self.lifecycle.clear()
        self.pending.clear()

    def stop(self):
        with self.lock:
            p, t = self.proc, self.thread
            if not p or p.poll() is not None:
                if self.state != 'stopped':
                    self._set('stopped', '', 'none')
                return True, 'was not running'
            self.stop_requested = True
            self._set('stopping', '')
        self._note('shutting the autonomy launch down (Ctrl-C) ...')
        for sig, wait in ((signal.SIGINT, self.stop_timeout), (signal.SIGTERM, 8.0), (signal.SIGKILL, 5.0)):
            try:
                os.killpg(os.getpgid(p.pid), sig)
            except (ProcessLookupError, PermissionError):
                break
            end = time.monotonic() + wait
            while time.monotonic() < end and p.poll() is None:
                time.sleep(0.2)
            if p.poll() is not None:
                break
            self._note(f'still alive after {wait:.0f} s, escalating', Log.WARN)
        if t:
            t.join(5.0)
        return True, 'stopped'

    # ------------------------------------------------------------------ monitoring
    def _tick(self):
        now = time.monotonic()
        running = self.state in ('starting', 'running', 'stopping')
        if running:
            present = self._nodes_now() - self.baseline
            for fqn in present:
                self.seen[fqn] = now
            if self.state == 'starting' and self.proc and self.proc.poll() is None and \
                    now - self.started_at > 6.0 and self.seen:
                self._set('running', '')
            self._poll_lifecycle(present)
        self._publish_diag(now)
        self._publish_state()

    def _poll_lifecycle(self, present):
        services = dict(self.get_service_names_and_types())
        for fqn in list(present):
            srv = f'{fqn}/get_state'
            if srv not in services or 'lifecycle_msgs/srv/GetState' not in services[srv]:
                continue
            if fqn not in self.lc_clients:
                self.lc_clients[fqn] = self.create_client(GetState, srv, callback_group=self.cb)
            fut = self.pending.get(fqn)
            if fut and fut[0].done():
                try:
                    self.lifecycle[fqn] = (fut[0].result().current_state.label, time.monotonic())
                except Exception:
                    pass
                del self.pending[fqn]
            elif fut and time.monotonic() - fut[1] > 5.0:
                del self.pending[fqn]
            if fqn not in self.pending and self.lc_clients[fqn].service_is_ready():
                self.pending[fqn] = (self.lc_clients[fqn].call_async(GetState.Request()), time.monotonic())

    def _publish_diag(self, now):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        level = {'running': DiagnosticStatus.OK, 'stopped': DiagnosticStatus.OK,
                 'crashed': DiagnosticStatus.ERROR}.get(self.state, DiagnosticStatus.WARN)
        present = self._nodes_now() - self.baseline if self.state != 'stopped' else set()
        missing = sorted(set(self.seen) - present) if self.state in ('running', 'starting') else []
        if missing and self.state == 'running':
            level = max(level, DiagnosticStatus.WARN)
        text = self.state + (f': {self.detail}' if self.detail else '')
        if self.state != 'stopped':
            text = f'{self.mode} - {text}'
        st = DiagnosticStatus(level=level, name='Autonomy launch', hardware_id='nav_launch', message=text)
        up = now - self.started_at if self.started_at and self.state in ('starting', 'running') else 0
        for k, v in (('state', self.state), ('mode', self.mode), ('uptime_s', f'{up:.0f}'),
                     ('nodes started', len(self.seen)), ('nodes missing', ', '.join(missing)),
                     ('map_name', self.get_parameter('map_name').value or '(auto / latest)'),
                     ('last exit code', '' if self.last_exit is None else self.last_exit),
                     ('last error line', self.last_error)):
            st.values.append(KeyValue(key=k, value=str(v)))
        arr.status.append(st)
        for fqn in sorted(self.seen):
            here = fqn in present
            lc = self.lifecycle.get(fqn)
            if not here:
                lvl, msg = DiagnosticStatus.ERROR, 'missing (node disappeared)'
            elif lc:
                lvl, msg = STATE_LEVEL.get(lc[0], DiagnosticStatus.WARN), lc[0]
            else:
                lvl, msg = DiagnosticStatus.OK, 'alive'
            arr.status.append(DiagnosticStatus(level=lvl, name=fqn, hardware_id='nav_launch', message=msg))
            if fqn in self.lc_clients:
                label = lc[0] if lc else 'no answer yet'
                arr.status.append(DiagnosticStatus(
                    level=STATE_LEVEL.get(label, DiagnosticStatus.WARN) if here else DiagnosticStatus.ERROR,
                    name=fqn, hardware_id='nav_lifecycle', message=label if here else 'missing'))
        self.diag_pub.publish(arr)

    # ------------------------------------------------------------------ services
    def _svc_start(self, mode, res):
        res.success, res.message = self.start(mode)
        return res

    def _svc_stop(self, req, res):
        res.success, res.message = self.stop()
        return res

    def shutdown(self):
        try:
            self.stop()
        except Exception:
            if self.proc and self.proc.poll() is None:
                try:
                    os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
                except Exception:
                    pass


def main():
    rclpy.init()
    node = AutonomyManager()
    ex = MultiThreadedExecutor(num_threads=6)
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
