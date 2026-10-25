#!/usr/bin/env python3
"""
Aggressive Fuzzy Logic PID Gain Scheduler (5Hz)

Independent 4-wheel fuzzy inference using Error and Error-Rate.
- Subscribes to geometry_msgs/msg/TwistStamped for /cmd_vel.
- Completely ignores Fuzzy boost during straight-line travel (protects cruising).
- Highly aggressive tuning bounds (Kp max 8.0) to instantly break stiction and force direction changes.
- Includes LPF and Deadband to protect the ESP32 serial line from micro-chatter.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

WHEEL_JOINTS = [
    'front_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_left_wheel_joint',
    'rear_right_wheel_joint',
]
LEFT_IDX = (0, 2)
RIGHT_IDX = (1, 3)
N = 4

def trimf(x, a, b, c):
    if x <= a or x >= c: return 0.0
    if a < x <= b: return (x - a) / (b - a) if b != a else 1.0
    if b < x < c: return (c - x) / (c - b) if b != c else 1.0
    return 0.0

def trapmf(x, a, b, c, d):
    if x <= a or x >= d: return 0.0
    if a < x <= b: return (x - a) / (b - a) if b != a else 1.0
    if b < x <= c: return 1.0
    if c < x < d: return (d - x) / (d - c) if c != d else 1.0
    return 0.0

class FuzzyGainScheduler(Node):
    def __init__(self):
        super().__init__('fuzzy_gain_scheduler')

        # 1. Robot Kinematics (65mm dia, 22cm track)
        self.declare_parameter('wheel_radius', 0.0325)   
        self.declare_parameter('wheel_separation', 0.22) 
        self.declare_parameter('cmd_vel_timeout', 0.5)

        # 2. Filter & Deadzone 
        self.declare_parameter('lpf_alpha', 0.75)       # 0.8 = Very fast response, minor smoothing
        self.declare_parameter('gain_deadzone', 0.15)  # Prevents serial micro-chatter

        # 3. Aggressive PID Limits (Max Kp increased to 8.0 to force heavy turns)
        self.declare_parameter('kp_min', [1.5, 1.5, 1.5, 1.5])
        self.declare_parameter('kp_max', [18.0, 18.0, 18.0, 18.0])
        self.declare_parameter('ki_min', [0.5, 0.5, 0.5, 0.5])
        self.declare_parameter('ki_max', [4.0, 4.0, 4.0, 4.0])
        self.declare_parameter('kd_min', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('kd_max', [0.3, 0.3, 0.3, 0.3])

        p = self.get_parameter
        self._radius = float(p('wheel_radius').value)
        self._half_track = 0.5 * float(p('wheel_separation').value)
        self._timeout = float(p('cmd_vel_timeout').value)
        self._lpf_alpha = float(p('lpf_alpha').value)
        self._deadzone = float(p('gain_deadzone').value)

        self._kp_min = list(p('kp_min').value)
        self._kp_max = list(p('kp_max').value)
        self._ki_min = list(p('ki_min').value)
        self._ki_max = list(p('ki_max').value)
        self._kd_min = list(p('kd_min').value)
        self._kd_max = list(p('kd_max').value)

        # State tracking
        self._cmd_v = 0.0
        self._cmd_w = 0.0
        self._cmd_time = None
        self._actual_speeds = [0.0] * N
        self._prev_error = [0.0] * N
        self._prev_time = [self.get_clock().now().nanoseconds * 1e-9] * N

        self._filtered_kp = list(self._kp_min)
        self._filtered_ki = list(self._ki_min)
        self._filtered_kd = list(self._kd_min)
        
        self._published_kp = list(self._kp_min)
        self._published_ki = list(self._ki_min)
        self._published_kd = list(self._kd_min)

        # I/O
        self.create_subscription(TwistStamped, '/agrobot_base_controller/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, qos_profile_sensor_data)
        
        self._kp_pub = self.create_publisher(Float64MultiArray, '/kp_controller/commands', 10)
        self._ki_pub = self.create_publisher(Float64MultiArray, '/ki_controller/commands', 10)
        self._kd_pub = self.create_publisher(Float64MultiArray, '/kd_controller/commands', 10)
        
        # Strict 5Hz Timer
        self.create_timer(0.2, self._tick)
        self.get_logger().info("Aggressive 5Hz Fuzzy Scheduler Initialized (TwistStamped).")

    def _on_cmd_vel(self, msg: TwistStamped):
        self._cmd_v = msg.twist.linear.x
        self._cmd_w = msg.twist.angular.z
        self._cmd_time = self.get_clock().now()

    def _on_joint_states(self, msg: JointState):
        if len(msg.velocity) != len(msg.name): return
        vel = dict(zip(msg.name, msg.velocity))
        for i, joint in enumerate(WHEEL_JOINTS):
            if joint in vel:
                self._actual_speeds[i] = vel[joint]

    def _compute_fuzzy_boost(self, e, de):
        # 1. Aggressive Error Memberships (Triggers instantly on small lags)
        E_Small  = trapmf(e, -1.0, 0.0, 0.1, 0.25)
        E_Medium = trimf(e, 0.15, 0.5, 1.0)
        E_Large  = trapmf(e, 0.7, 1.2, 10.0, 10.0)

        # 2. Delta Error Memberships
        dE_Falling = trapmf(de, -10.0, -10.0, -0.2, -0.05)
        dE_Steady  = trimf(de, -0.1, 0.0, 0.1)
        dE_Rising  = trapmf(de, 0.05, 0.2, 10.0, 10.0)

        # 3. Rule Weights (Biased heavily upward to ensure Kp snaps > 4.5)
        rules = [
            (E_Small, 0.0),
            (min(E_Medium, dE_Falling), 0.4),
            (min(E_Medium, dE_Steady), 0.8),
            (min(E_Medium, dE_Rising), 1.0),
            (min(E_Large, dE_Falling), 0.7),
            (min(E_Large, max(dE_Steady, dE_Rising)), 1.0)
        ]

        # 4. Defuzzification
        num = sum(weight * output for weight, output in rules)
        den = sum(weight for weight, output in rules)
        return num / den if den > 0.001 else 0.0

    def _tick(self):
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        # Handle Timeout
        v, w = self._cmd_v, self._cmd_w
        if self._cmd_time is None or (now - self._cmd_time).nanoseconds * 1e-9 > self._timeout:
            v = w = 0.0

        # Kinematic Speed Calculation
        left_target = (v - w * self._half_track) / self._radius
        right_target = (v + w * self._half_track) / self._radius
        targets = [0.0] * N
        for i in LEFT_IDX: targets[i] = left_target
        for i in RIGHT_IDX: targets[i] = right_target

        requires_publish = False
        boosts = [0.0] * N
        
        # Turn Intent Gate: Protects straight line motion from fuzzy bursts
        is_turning = abs(w) > 0.15 

        for i in range(N):
            e = abs(targets[i] - self._actual_speeds[i])
            dt = max(0.001, now_s - self._prev_time[i])
            de = (e - self._prev_error[i]) / dt
            
            self._prev_error[i] = e
            self._prev_time[i] = now_s

            # Calculate crisp outputs
            if is_turning:
                boost = self._compute_fuzzy_boost(e, de)
            else:
                boost = 0.0

            boosts[i] = boost

            raw_kp = self._kp_min[i] + boost * (self._kp_max[i] - self._kp_min[i])
            raw_ki = self._ki_min[i] + boost * (self._ki_max[i] - self._ki_min[i])
            raw_kd = self._kd_min[i] + boost * (self._kd_max[i] - self._kd_min[i])

            # Apply Low Pass Filter
            alpha = self._lpf_alpha
            self._filtered_kp[i] = (alpha * raw_kp) + ((1.0 - alpha) * self._filtered_kp[i])
            self._filtered_ki[i] = (alpha * raw_ki) + ((1.0 - alpha) * self._filtered_ki[i])
            self._filtered_kd[i] = (alpha * raw_kd) + ((1.0 - alpha) * self._filtered_kd[i])

            # Apply Deadzone Quantization
            if abs(self._filtered_kp[i] - self._published_kp[i]) > self._deadzone or \
               abs(self._filtered_ki[i] - self._published_ki[i]) > self._deadzone or \
               abs(self._filtered_kd[i] - self._published_kd[i]) > self._deadzone:
                
                self._published_kp[i] = round(self._filtered_kp[i], 2)
                self._published_ki[i] = round(self._filtered_ki[i], 2)
                self._published_kd[i] = round(self._filtered_kd[i], 2)
                requires_publish = True

        if requires_publish:
            self._kp_pub.publish(Float64MultiArray(data=self._published_kp))
            self._ki_pub.publish(Float64MultiArray(data=self._published_ki))
            self._kd_pub.publish(Float64MultiArray(data=self._published_kd))
        
        mode_str = "ACTIVE" if is_turning else "INACTIVE (Straight)"
#        self.get_logger().info(
#            f"\n--- 5Hz FUZZY TICK ---"
#            f"\nCmd   : v={v:.2f}, w={w:.2f} | Turn Mode: {mode_str}"
#            f"\nBoost : {[round(b, 2) for b in boosts]}"
#            f"\nKp    : {self._published_kp}"
#            f"\nKi    : {self._published_ki}"
#            f"\nKd    : {self._published_kd}"
#        )

def main(args=None):
    rclpy.init(args=args)
    node = FuzzyGainScheduler()
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
