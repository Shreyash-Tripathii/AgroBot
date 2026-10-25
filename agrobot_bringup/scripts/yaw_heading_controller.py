#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Imu


def euler_yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw (rotation around Z-axis in radians) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class YawHeadingController(Node):
    def __init__(self):
        super().__init__('yaw_heading_controller')

        # Declare tunable parameters
        self.declare_parameter('kp', 2.2)
        self.declare_parameter('kd', 0.15)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('max_angular_vel', 2.0)       # rad/s clamp
        self.declare_parameter('max_heading_error', 0.785)   # ~45 deg anti-windup clamp
        self.declare_parameter('cmd_timeout', 0.5)           # seconds
        self.declare_parameter('input_stamped', False)       # True if input /cmd_vel is TwistStamped
        self.declare_parameter('frame_id', 'base_link')

        # State variables
        self.current_yaw = None
        self.current_gyro_z = 0.0
        self.target_yaw = None

        self.cmd_linear_x = 0.0
        self.cmd_linear_y = 0.0
        self.cmd_angular_z = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_loop_time = self.get_clock().now()
        self.integral_error = 0.0

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_game',
            self.imu_callback,
            10
        )

        if self.get_parameter('input_stamped').value:
            self.cmd_sub = self.create_subscription(
                TwistStamped, '/cmd_vel', self.cmd_stamped_callback, 10
            )
        else:
            self.cmd_sub = self.create_subscription(
                Twist, '/cmd_vel', self.cmd_Callback, 10
            )

        # Publisher to ros2_control diff_drive / base controller
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/agrobot_base_controller/cmd_vel',
            10
        )

        # 50 Hz control loop (0.02s)
        self.timer = self.create_timer(0.02, self.control_loop)
        self.get_logger().info('Yaw Heading Controller initialized and waiting for /imu/data_game...')

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        self.current_yaw = euler_yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.current_gyro_z = msg.angular_velocity.z

        # Lock initial target heading on startup
        if self.target_yaw is None:
            self.target_yaw = self.current_yaw
            self.get_logger().info(f'Initial heading locked at: {math.degrees(self.target_yaw):.2f} deg')

    def cmd_Callback(self, msg: Twist):
        self.cmd_linear_x = msg.linear.x
        self.cmd_linear_y = msg.linear.y
        self.cmd_angular_z = msg.angular.z
        self.last_cmd_time = self.get_clock().now()

    def cmd_stamped_callback(self, msg: TwistStamped):
        self.cmd_linear_x = msg.twist.linear.x
        self.cmd_linear_y = msg.twist.linear.y
        self.cmd_angular_z = msg.twist.angular.z
        self.last_cmd_time = self.get_clock().now()

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_loop_time).nanoseconds * 1e-9
        self.last_loop_time = now

        if self.current_yaw is None or dt <= 0.0 or dt > 0.5:
            return

        # Check for command timeout (stop robot & reset target heading to current if idle)
        time_since_cmd = (now - self.last_cmd_time).nanoseconds * 1e-9
        is_stopped = (
            abs(self.cmd_linear_x) < 1e-4 and
            abs(self.cmd_linear_y) < 1e-4 and
            abs(self.cmd_angular_z) < 1e-4
        )

        if time_since_cmd > self.get_parameter('cmd_timeout').value or is_stopped:
            # While completely stopped, sync target_yaw to current_yaw so external pushes
            # don't cause the robot to violently snap back when neither linear nor angular vel is requested.
            # (Remove `self.target_yaw = self.current_yaw` if you want active station-keeping while idle).
            self.target_yaw = self.current_yaw
            self.integral_error = 0.0
            self.publish_twist(0.0, 0.0, 0.0, now)
            return

        # 1. Integrate desired angular velocity into target yaw
        self.target_yaw = normalize_angle(self.target_yaw + self.cmd_angular_z * dt)

        # 2. Compute shortest-path heading error
        yaw_error = normalize_angle(self.target_yaw - self.current_yaw)

        # Prevent massive windup if robot gets physically blocked
        max_err = self.get_parameter('max_heading_error').value
        if abs(yaw_error) > max_err:
            self.target_yaw = normalize_angle(self.current_yaw + math.copysign(max_err, yaw_error))
            yaw_error = math.copysign(max_err, yaw_error)

        # 3. PID + Feed-Forward Calculation
        kp = self.get_parameter('kp').value
        kd = self.get_parameter('kd').value
        ki = self.get_parameter('ki').value

        self.integral_error = max(min(self.integral_error + yaw_error * dt, 0.5), -0.5)
        rate_error = self.cmd_angular_z - self.current_gyro_z

        corrected_angular_z = (
            self.cmd_angular_z +
            (kp * yaw_error) +
            (kd * rate_error) +
            (ki * self.integral_error)
        )

        # 4. Clamp output angular velocity
        max_w = self.get_parameter('max_angular_vel').value
        corrected_angular_z = max(min(corrected_angular_z, max_w), -max_w)

        # 5. Publish TwistStamped to /agrobot_base_controller/cmd_vel
        self.publish_twist(self.cmd_linear_x, self.cmd_linear_y, corrected_angular_z, now)

    def publish_twist(self, vx: float, vy: float, wz: float, timestamp):
        out_msg = TwistStamped()
        out_msg.header.stamp = timestamp.to_msg()
        out_msg.header.frame_id = self.get_parameter('frame_id').value
        out_msg.twist.linear.x = float(vx)
        out_msg.twist.linear.y = float(vy)
        out_msg.twist.linear.z = 0.0
        out_msg.twist.angular.x = 0.0
        out_msg.twist.angular.y = 0.0
        out_msg.twist.angular.z = float(wz)
        self.cmd_pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YawHeadingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
