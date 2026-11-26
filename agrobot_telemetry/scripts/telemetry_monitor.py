#!/usr/bin/env python3
"""
AgroBot telemetry monitor: turns the robot's state into things Foxglove (or any tool) can plot
and alarm on. Runs next to the foxglove_bridge (robot side or PC, anywhere on the ROS graph).

Publishes
  /diagnostics                  one DiagnosticStatus per watched sensor stream (rate / age),
                                System (CPU, memory, temperature), Controllers, Hardware
  /telemetry/wheel_velocity     Float64MultiArray [FL, FR, RL, RR] rad/s  (fixed order, from /joint_states)
  /telemetry/arm_position       Float64MultiArray [yaw, shoulder, wrist] rad
  /telemetry/heading_deg        Float64, yaw of /odometry/filtered in degrees
  /telemetry/cpu_percent, /telemetry/mem_percent, /telemetry/cpu_temp_c   Float64

Fixed-order arrays exist because the order of /joint_states differs between the simulation and the
real ESP32 hardware, so plots cannot index it directly.
"""
import glob
import importlib
import math
import os
import time

import rclpy
from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'rear_left_wheel_joint', 'rear_right_wheel_joint']
ARM = ['arm_base_yaw_joint', 'arm_shoulder_pitch_joint', 'arm_wrist_pitch_joint']

# name|topic|type|expected_hz|required   (optional streams report STALE instead of ERROR)
DEFAULT_WATCH = [
    'LiDAR scan|/scan|sensor_msgs/msg/LaserScan|8|true',
    'IMU game|/imu/data_game|sensor_msgs/msg/Imu|50|true',
    'EKF odometry|/odometry/filtered|nav_msgs/msg/Odometry|50|true',
    'Joint states|/joint_states|sensor_msgs/msg/JointState|50|true',
    'IMU absolute|/imu/data_mag|sensor_msgs/msg/Imu|5|false',
    'GPS fix|/gps/fix|sensor_msgs/msg/NavSatFix|5|false',
    'Arm camera|/camera/image_raw|sensor_msgs/msg/Image|15|false',
]

OK, WARN, ERROR, STALE = (DiagnosticStatus.OK, DiagnosticStatus.WARN,
                          DiagnosticStatus.ERROR, DiagnosticStatus.STALE)


def import_msg(type_str):
    pkg, _, name = type_str.split('/')
    return getattr(importlib.import_module(f'{pkg}.msg'), name)


def status(name, level, message, values=None):
    s = DiagnosticStatus()
    s.name, s.level, s.message, s.hardware_id = name, level, message, 'agrobot'
    for k, v in (values or {}).items():
        s.values.append(KeyValue(key=str(k), value=str(v)))
    return s


class Watch:
    def __init__(self, label, topic, expected, required):
        self.label, self.topic, self.expected, self.required = label, topic, expected, required
        self.count = 0
        self.window_start = time.monotonic()
        self.last_rx = None
        self.hz = 0.0

    def tick(self):
        self.count += 1
        self.last_rx = time.monotonic()

    def update(self):
        now = time.monotonic()
        dt = max(now - self.window_start, 1e-3)
        self.hz = self.count / dt
        self.count, self.window_start = 0, now


class TelemetryMonitor(Node):
    def __init__(self):
        super().__init__('telemetry_monitor')
        self.declare_parameter('watch', DEFAULT_WATCH)
        self.declare_parameter('inactive_ok', ['arm_position_controller'])
        # RPi 4/5 start throttling at 80 C; laptops run hotter, so override on a PC if you like
        self.declare_parameter('temp_warn_c', 80.0)
        self.declare_parameter('temp_error_c', 90.0)

        self.watches = []
        for entry in self.get_parameter('watch').value:
            label, topic, type_str, hz, required = entry.split('|')
            w = Watch(label, topic, float(hz), required.strip().lower() == 'true')
            self.watches.append(w)
            # raw=True: count messages without deserialising (cheap even for images)
            self.create_subscription(import_msg(type_str), topic, lambda _m, w=w: w.tick(),
                                     qos_profile_sensor_data, raw=True)

        self.joints = {}
        self.heading = float('nan')
        self.create_subscription(JointState, '/joint_states', self._on_joints, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, qos_profile_sensor_data)

        self.pub_diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.pub_wheel = self.create_publisher(Float64MultiArray, '/telemetry/wheel_velocity', 10)
        self.pub_arm = self.create_publisher(Float64MultiArray, '/telemetry/arm_position', 10)
        self.pub_heading = self.create_publisher(Float64, '/telemetry/heading_deg', 10)
        self.pub_cpu = self.create_publisher(Float64, '/telemetry/cpu_percent', 10)
        self.pub_mem = self.create_publisher(Float64, '/telemetry/mem_percent', 10)
        self.pub_temp = self.create_publisher(Float64, '/telemetry/cpu_temp_c', 10)

        self.cli_ctrl = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.cli_hw = self.create_client(ListHardwareComponents, '/controller_manager/list_hardware_components')
        self.controllers = None   # list of (name, state) or None = unreachable
        self.hardware = None
        self._ctrl_future = self._hw_future = None

        self._cpu_prev = self._read_cpu()
        self.create_timer(0.1, self._fast)        # 10 Hz plot topics
        self.create_timer(1.0, self._slow)        # 1 Hz diagnostics
        self.create_timer(2.0, self._poll_services)
        self.get_logger().info(f'Telemetry monitor up, watching {len(self.watches)} streams.')

    # ---- callbacks ---------------------------------------------------------------
    def _on_joints(self, msg):
        self.joints = {n: (p, v) for n, p, v in zip(
            msg.name, msg.position if msg.position else [float('nan')] * len(msg.name),
            msg.velocity if msg.velocity else [float('nan')] * len(msg.name))}

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.heading = math.degrees(yaw)

    # ---- 10 Hz plot topics ----------------------------------------------------------
    def _fast(self):
        nan = float('nan')
        self.pub_wheel.publish(Float64MultiArray(data=[self.joints.get(j, (nan, nan))[1] for j in WHEELS]))
        self.pub_arm.publish(Float64MultiArray(data=[self.joints.get(j, (nan, nan))[0] for j in ARM]))
        self.pub_heading.publish(Float64(data=self.heading))

    # ---- system metrics -----------------------------------------------------------------
    @staticmethod
    def _read_cpu():
        try:
            with open('/proc/stat') as f:
                v = [int(x) for x in f.readline().split()[1:]]
            return sum(v), v[3] + (v[4] if len(v) > 4 else 0)   # total, idle+iowait
        except OSError:
            return None

    def _cpu_percent(self):
        cur = self._read_cpu()
        if cur is None or self._cpu_prev is None:
            return float('nan')
        dt, di = cur[0] - self._cpu_prev[0], cur[1] - self._cpu_prev[1]
        self._cpu_prev = cur
        return 100.0 * (dt - di) / dt if dt > 0 else float('nan')

    @staticmethod
    def _mem_percent():
        try:
            info = {}
            with open('/proc/meminfo') as f:
                for line in f:
                    k, v = line.split(':')
                    info[k] = int(v.split()[0])
            return 100.0 * (1.0 - info['MemAvailable'] / info['MemTotal'])
        except (OSError, KeyError, ValueError):
            return float('nan')

    @staticmethod
    def _temp_c():
        best = float('nan')
        for p in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
            try:
                with open(p) as f:
                    t = int(f.read().strip()) / 1000.0
                best = t if math.isnan(best) else max(best, t)
            except (OSError, ValueError):
                pass
        return best

    # ---- controller manager polling ----------------------------------------------------------
    def _poll_services(self):
        for cli, attr, req_t in ((self.cli_ctrl, '_ctrl_future', ListControllers.Request),
                                 (self.cli_hw, '_hw_future', ListHardwareComponents.Request)):
            old = getattr(self, attr)
            if old is not None and not old.done():
                continue
            if cli.service_is_ready():
                setattr(self, attr, cli.call_async(req_t()))
            else:
                setattr(self, attr, None)
                if cli is self.cli_ctrl:
                    self.controllers = None
                else:
                    self.hardware = None

    def _harvest(self):
        if self._ctrl_future is not None and self._ctrl_future.done():
            try:
                self.controllers = [(c.name, c.state) for c in self._ctrl_future.result().controller]
            except Exception:  # noqa: BLE001
                self.controllers = None
        if self._hw_future is not None and self._hw_future.done():
            try:
                self.hardware = [(c.name, c.state.label) for c in self._hw_future.result().component]
            except Exception:  # noqa: BLE001
                self.hardware = None

    # ---- 1 Hz diagnostics -------------------------------------------------------------------------
    def _slow(self):
        self._harvest()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        now = time.monotonic()

        # System
        cpu, mem, temp = self._cpu_percent(), self._mem_percent(), self._temp_c()
        self.pub_cpu.publish(Float64(data=cpu))
        self.pub_mem.publish(Float64(data=mem))
        self.pub_temp.publish(Float64(data=temp))
        level, msg = OK, 'OK'
        tw, te = self.get_parameter('temp_warn_c').value, self.get_parameter('temp_error_c').value
        if not math.isnan(temp) and temp >= te:
            level, msg = ERROR, f'CPU temperature {temp:.0f} C'
        elif (not math.isnan(temp) and temp >= tw) or cpu > 90 or mem > 90:
            level, msg = WARN, 'High load / temperature'
        load1 = os.getloadavg()[0] if hasattr(os, 'getloadavg') else float('nan')
        arr.status.append(status('System', level, msg, {
            'cpu %': f'{cpu:.1f}', 'memory %': f'{mem:.1f}',
            'cpu temp C': 'n/a' if math.isnan(temp) else f'{temp:.1f}', 'load 1m': f'{load1:.2f}'}))

        # Sensor streams
        for w in self.watches:
            w.update()
            age = None if w.last_rx is None else now - w.last_rx
            vals = {'topic': w.topic, 'rate Hz': f'{w.hz:.1f}', 'expected Hz': f'{w.expected:g}',
                    'age s': 'never' if age is None else f'{age:.1f}'}
            if age is None or age > 3.0:
                lvl, m = (ERROR, 'no data') if w.required else (STALE, 'not publishing')
            elif w.hz < 0.7 * w.expected:
                lvl, m = WARN, f'slow: {w.hz:.1f} Hz'
            else:
                lvl, m = OK, f'{w.hz:.1f} Hz'
            arr.status.append(status(f'Sensor: {w.label}', lvl, m, vals))

        # Controllers
        inactive_ok = set(self.get_parameter('inactive_ok').value)
        if self.controllers is None:
            arr.status.append(status('Controllers', STALE, 'controller_manager not reachable'))
        else:
            bad = [n for n, s in self.controllers if s != 'active' and not (s == 'inactive' and n in inactive_ok)]
            arr.status.append(status('Controllers', ERROR if bad else OK,
                                     f'not active: {", ".join(bad)}' if bad else f'{len(self.controllers)} loaded, all as expected',
                                     dict(sorted(self.controllers))))
        if self.hardware is None:
            arr.status.append(status('Hardware', STALE, 'controller_manager not reachable'))
        else:
            bad = [n for n, s in self.hardware if s != 'active']
            arr.status.append(status('Hardware', ERROR if bad else OK,
                                     f'not active: {", ".join(bad)}' if bad else 'all components active',
                                     dict(sorted(self.hardware))))
        self.pub_diag.publish(arr)


def main():
    rclpy.init()
    node = TelemetryMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
