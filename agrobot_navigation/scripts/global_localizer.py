#!/usr/bin/env python3
"""
global_localizer.py

Linear startup state machine for AMCL global localization (ROS 2 Jazzy):

    SCATTER -> SPIN -> MONITOR -> LOCALIZED      (or FAILED)

  SCATTER  : call /reinitialize_global_localization (std_srvs/Empty) so AMCL
             spreads particles over the whole map.
  SPIN     : slow, open-loop, time-based in-place rotation (spin_angle_deg
             at spin_speed_rad_s) published as a Twist on /cmd_vel. Your yaw
             heading controller subscribes to that topic and does the IMU
             feedback / drift correction downstream.
  MONITOR  : robot stopped; wait until /amcl_pose covariance (x, y, yaw) is
             below the thresholds for N consecutive fresh messages. If it does
             not converge in time, re-scatter and retry (max_retries).
  LOCALIZED: zero velocity is sent, the timer is cancelled and the node exits
             with code 0 so Nav2 can take over. FAILED exits with code 1.

Run (after map_server + amcl are ACTIVE):
    python3 global_localizer.py --ros-args -p spin_speed_rad_s:=0.25

Frames: this node never touches TF. The only frame name used is the
TwistStamped header.frame_id, which defaults to 'base_footprint'.
"""

import enum
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from std_srvs.srv import Empty


class State(enum.Enum):
    SCATTER = 'SCATTER'
    SPIN = 'SPIN'
    MONITOR = 'MONITOR'
    LOCALIZED = 'LOCALIZED'
    FAILED = 'FAILED'


class GlobalLocalizer(Node):

    def __init__(self):
        super().__init__('global_localizer')

        # ------------------------------------------------------------------
        # Parameters (all tunable at launch: --ros-args -p name:=value)
        # ------------------------------------------------------------------
        # Control loop rate [Hz].
        self.declare_parameter('loop_rate_hz', 10.0)

        # Topics / services.
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')  # input of the yaw heading controller
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('reinit_service', '/reinitialize_global_localization')

        # The yaw heading controller subscribes to /cmd_vel as a plain Twist
        # (it alone publishes to /agrobot_base_controller/cmd_vel), so this is
        # False. Set True only if /cmd_vel expects TwistStamped.
        self.declare_parameter('use_stamped_twist', False)
        self.declare_parameter('cmd_vel_frame', 'base_footprint')  # TwistStamped header.frame_id

        # Service handling.
        self.declare_parameter('service_wait_timeout_s', 30.0)  # wait for AMCL service
        self.declare_parameter('service_call_timeout_s', 5.0)   # wait for the response

        # Spin. Keep it SLOW: AMCL's likelihood field and open-loop wheel
        # slip both get worse at speed. Sign of speed sets direction.
        self.declare_parameter('spin_speed_rad_s', 0.3)
        # Total rotation. 360 = one full turn. Use e.g. 400 for margin against
        # wheel slip (open loop, time based: t = angle / speed).
        self.declare_parameter('spin_angle_deg', 360.0)


        # Convergence thresholds on the diagonal of the 6x6 covariance.
        # Units: x, y in m^2 ; yaw in rad^2.   (sigma = sqrt(value))
        #   0.03 m^2     -> ~17 cm sigma
        #   0.03 rad^2   -> ~10 deg sigma
        # Tighten for precise docking, loosen if AMCL never gets below them
        # in your environment (long corridors / symmetric maps).
        self.declare_parameter('x_cov_threshold', 0.86)
        self.declare_parameter('y_cov_threshold', 0.36)
        self.declare_parameter('yaw_cov_threshold', 0.09)

        # Require this many consecutive fresh /amcl_pose messages under the
        # thresholds before declaring success (rejects one-off lucky updates).
        self.declare_parameter('required_consecutive', 3)

        # MONITOR: max time to wait after the spin before giving up on this
        # attempt, and how many extra scatter+spin attempts are allowed.
        self.declare_parameter('monitor_timeout_s', 10.0)
        self.declare_parameter('max_retries', 2)

        # If true, finish as soon as covariance converges during the spin
        # instead of completing the full rotation. Default off: finishing
        # the full turn disambiguates symmetric environments.
        self.declare_parameter('allow_early_exit', False)

        p = self.get_parameter
        self.loop_rate = float(p('loop_rate_hz').value)
        self.cmd_vel_topic = p('cmd_vel_topic').value
        self.pose_topic = p('amcl_pose_topic').value
        self.reinit_srv_name = p('reinit_service').value
        self.use_stamped = bool(p('use_stamped_twist').value)
        self.cmd_vel_frame = p('cmd_vel_frame').value
        self.srv_wait_timeout = float(p('service_wait_timeout_s').value)
        self.srv_call_timeout = float(p('service_call_timeout_s').value)
        self.spin_speed = float(p('spin_speed_rad_s').value)
        self.spin_angle = abs(math.radians(float(p('spin_angle_deg').value)))
        self.thr_x = float(p('x_cov_threshold').value)
        self.thr_y = float(p('y_cov_threshold').value)
        self.thr_yaw = float(p('yaw_cov_threshold').value)
        self.required_consecutive = int(p('required_consecutive').value)
        self.monitor_timeout = float(p('monitor_timeout_s').value)
        self.max_retries = int(p('max_retries').value)
        self.allow_early_exit = bool(p('allow_early_exit').value)

        if self.spin_speed == 0.0:
            raise ValueError('spin_speed_rad_s must be non-zero')
        self.spin_duration = self.spin_angle / abs(self.spin_speed)

        # ------------------------------------------------------------------
        # ROS interfaces
        # ------------------------------------------------------------------
        msg_type = TwistStamped if self.use_stamped else Twist
        self._cmd_pub = self.create_publisher(msg_type, self.cmd_vel_topic, 10)

        # Volatile + reliable: AMCL publishes /amcl_pose as transient_local,
        # and a volatile subscriber is compatible with that. This avoids being
        # handed a stale latched pose from before the scatter.
        pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self._pose_cb, pose_qos)

        self._reinit_client = self.create_client(Empty, self.reinit_srv_name)

        # ------------------------------------------------------------------
        # State
        # ------------------------------------------------------------------
        self.state = State.SCATTER
        self.finished = False
        self._attempt = 0
        self._state_enter_ns = self._now_ns()
        self._scatter_future = None
        self._scatter_call_ns = 0
        self._scatter_done_ns = 0       # poses stamped before this are stale
        self._consecutive_ok = 0
        self._last_cov = (float('nan'),) * 3

        self._timer = self.create_timer(1.0 / self.loop_rate, self._tick)
        self.get_logger().info(
            f'Global localizer started. Spin: {math.degrees(self.spin_angle):.0f} deg '
            f'@ {self.spin_speed:.2f} rad/s ({self.spin_duration:.1f} s). '
            f'Thresholds: x={self.thr_x}, y={self.thr_y}, yaw={self.thr_yaw}')

    # ----------------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------------
    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _elapsed_s(self) -> float:
        return (self._now_ns() - self._state_enter_ns) / 1e9

    def _transition(self, new_state: State):
        self.get_logger().info(f'State: {self.state.value} -> {new_state.value}')
        self.state = new_state
        self._state_enter_ns = self._now_ns()

    def _publish_cmd(self, angular_z: float):
        if self.use_stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.cmd_vel_frame
            msg.twist.angular.z = angular_z
        else:
            msg = Twist()
            msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def stop_robot(self, repeats: int = 5):
        """Publish zero velocity a few times (a single message can be lost)."""
        try:
            for _ in range(repeats):
                self._publish_cmd(0.0)
        except Exception:  # context may already be shut down
            pass

    def _finish(self, final_state: State):
        """Stop motors, cancel the timer and flag the main loop to exit."""
        self.stop_robot()
        self._transition(final_state)
        self._timer.cancel()
        self.finished = True

    # ----------------------------------------------------------------------
    # /amcl_pose callback: covariance bookkeeping only
    # ----------------------------------------------------------------------
    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        if self.state not in (State.SPIN, State.MONITOR):
            return

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns < self._scatter_done_ns:
            return  # estimate predates the particle reset -> ignore

        # 6x6 row-major covariance over [x, y, z, roll, pitch, yaw]:
        # x = [0], y = [7], yaw = [35].
        c = msg.pose.covariance
        cx, cy, cyaw = c[0], c[7], c[35]
        self._last_cov = (cx, cy, cyaw)

        if not all(math.isfinite(v) for v in (cx, cy, cyaw)):
            self._consecutive_ok = 0
            return

        if cx < self.thr_x and cy < self.thr_y and cyaw < self.thr_yaw:
            self._consecutive_ok += 1
        else:
            self._consecutive_ok = 0

    # ----------------------------------------------------------------------
    # Main control loop
    # ----------------------------------------------------------------------
    def _tick(self):
        if self.finished:
            return
        handler = {
            State.SCATTER: self._do_scatter,
            State.SPIN: self._do_spin,
            State.MONITOR: self._do_monitor,
        }.get(self.state)
        if handler:
            handler()

    # --- SCATTER ----------------------------------------------------------
    def _do_scatter(self):
        if self._scatter_future is None:
            if not self._reinit_client.service_is_ready():
                if self._elapsed_s() > self.srv_wait_timeout:
                    self.get_logger().error(
                        f'Service {self.reinit_srv_name} unavailable '
                        f'after {self.srv_wait_timeout:.0f} s. Is AMCL active?')
                    self._finish(State.FAILED)
                else:
                    self.get_logger().info(
                        f'Waiting for {self.reinit_srv_name} ...',
                        throttle_duration_sec=2.0)
                return
            self.get_logger().info('Scattering particles (global localization)')
            self._scatter_future = self._reinit_client.call_async(Empty.Request())
            self._scatter_call_ns = self._now_ns()
            return

        if self._scatter_future.done():
            if self._scatter_future.exception() is not None:
                self.get_logger().error(
                    f'Scatter service call failed: {self._scatter_future.exception()}')
                self._finish(State.FAILED)
                return
            self._scatter_done_ns = self._now_ns()
            self._consecutive_ok = 0
            self._last_cov = (float('nan'),) * 3
            self._scatter_future = None
            self._transition(State.SPIN)
        elif (self._now_ns() - self._scatter_call_ns) / 1e9 > self.srv_call_timeout:
            self.get_logger().error('Scatter service call timed out')
            self._finish(State.FAILED)

    # --- SPIN -------------------------------------------------------------
    def _do_spin(self):
        if self.allow_early_exit and self._consecutive_ok >= self.required_consecutive:
            self.get_logger().info('Converged during spin, exiting early')
            self._finish(State.LOCALIZED)
            return

        if self._elapsed_s() >= self.spin_duration:
            self.stop_robot()
            self._transition(State.MONITOR)
            return

        self._publish_cmd(self.spin_speed)
        self.get_logger().info(
            f'Spinning {self._elapsed_s():.1f}/{self.spin_duration:.1f} s | '
            f'cov x={self._last_cov[0]:.4f} y={self._last_cov[1]:.4f} '
            f'yaw={self._last_cov[2]:.4f}',
            throttle_duration_sec=2.0)

    # --- MONITOR ----------------------------------------------------------
    def _do_monitor(self):
        self._publish_cmd(0.0)  # keep holding still

        if self._consecutive_ok >= self.required_consecutive:
            self.get_logger().info(
                f'Localized. cov x={self._last_cov[0]:.4f} '
                f'y={self._last_cov[1]:.4f} yaw={self._last_cov[2]:.4f}')
            self._finish(State.LOCALIZED)
            return

        self.get_logger().info(
            f'Waiting for convergence: cov x={self._last_cov[0]:.4f} '
            f'y={self._last_cov[1]:.4f} yaw={self._last_cov[2]:.4f} '
            f'({self._consecutive_ok}/{self.required_consecutive})',
            throttle_duration_sec=1.0)

        if self._elapsed_s() > self.monitor_timeout:
            if self._attempt < self.max_retries:
                self._attempt += 1
                self.get_logger().warn(
                    f'Not converged, retrying ({self._attempt}/{self.max_retries})')
                self._transition(State.SCATTER)
            else:
                self.get_logger().error('Global localization failed: no convergence')
                self._finish(State.FAILED)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalLocalizer()
    exit_code = 1
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = 0 if node.state == State.LOCALIZED else 1
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
