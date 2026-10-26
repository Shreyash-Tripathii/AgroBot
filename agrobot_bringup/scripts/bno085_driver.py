#!/usr/bin/env python3
import time

import board
import digitalio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import warnings
from adafruit_extended_bus import ExtendedI2C
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C


class Bno085Node(Node):
    def __init__(self):
        super().__init__("bno085_node")

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("i2c_bus", 3)
        self.declare_parameter("i2c_address", 0x4A)
        self.declare_parameter("use_reset_pin", True)
        self.declare_parameter("game_rate_hz", 100.0)
        self.declare_parameter("mag_rate_hz", 30.0)
        # variances (diagonal); tune for your EKF
        self.declare_parameter("orientation_var", 0.01)
        self.declare_parameter("gyro_var", 0.001)
        self.declare_parameter("accel_var", 0.05)

        self.frame_id = self.get_parameter("frame_id").value
        game_hz = self.get_parameter("game_rate_hz").value
        mag_hz = self.get_parameter("mag_rate_hz").value

        if self.get_parameter("use_reset_pin").value:
            self._hw_reset()
        warnings.filterwarnings("ignore", category=RuntimeWarning, module="adafruit_blinka")
        i2c = ExtendedI2C(self.get_parameter("i2c_bus").value)
        self.bno = BNO08X_I2C(i2c, address=self.get_parameter("i2c_address").value)

        game_us = int(1_000_000 / game_hz)   # 100 Hz -> 10000 us
        mag_us = int(1_000_000 / mag_hz)     # 30 Hz  -> 33333 us

        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER, game_us)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE, game_us)
        self.bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, game_us)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, mag_us)

        self.pub_game = self.create_publisher(Imu, "/imu/data_game", 10)
        self.pub_mag = self.create_publisher(Imu, "/imu/data_mag", 10)

        self.create_timer(1.0 / game_hz, self.publish_game)
        self.create_timer(1.0 / mag_hz, self.publish_mag)

        self.get_logger().info(
            f"BNO085 up: /imu/data_game @ {game_hz} Hz, /imu/data_mag @ {mag_hz} Hz, "
            f"frame '{self.frame_id}'"
        )

    def _hw_reset(self):
        rst = digitalio.DigitalInOut(board.D4)
        rst.direction = digitalio.Direction.OUTPUT
        rst.value = False
        time.sleep(0.01)
        rst.value = True
        time.sleep(0.5)

    def _build_msg(self, quat):
        qi, qj, qk, qr = quat
        ax, ay, az = self.bno.acceleration
        gx, gy, gz = self.bno.gyro

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        msg.orientation.x = float(qi)
        msg.orientation.y = float(qj)
        msg.orientation.z = float(qk)
        msg.orientation.w = float(qr)

        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)

        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)

        ov = self.get_parameter("orientation_var").value
        gv = self.get_parameter("gyro_var").value
        av = self.get_parameter("accel_var").value
        msg.orientation_covariance = [ov, 0.0, 0.0, 0.0, ov, 0.0, 0.0, 0.0, ov]
        msg.angular_velocity_covariance = [gv, 0.0, 0.0, 0.0, gv, 0.0, 0.0, 0.0, gv]
        msg.linear_acceleration_covariance = [av, 0.0, 0.0, 0.0, av, 0.0, 0.0, 0.0, av]
        return msg

    def publish_game(self):
        try:
            self.pub_game.publish(self._build_msg(self.bno.game_quaternion))
        except (OSError, RuntimeError) as e:
            self.get_logger().warn(f"game read failed: {e}", throttle_duration_sec=2.0)

    def publish_mag(self):
        try:
            # Rotation Vector = accel + gyro + magnetometer fusion (absolute heading)
            self.pub_mag.publish(self._build_msg(self.bno.quaternion))
        except (OSError, RuntimeError) as e:
            self.get_logger().warn(f"mag read failed: {e}", throttle_duration_sec=2.0)


def main():
    rclpy.init()
    node = Bno085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
