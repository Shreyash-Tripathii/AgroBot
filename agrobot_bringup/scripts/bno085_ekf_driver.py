#!/usr/bin/env python3
"""
bno085_shtp_node.py

Zero-dependency, pure-pyserial UART-SHTP driver for the Hillcrest BNO085 (3,000,000 baud).
Bypasses `adafruit_bno08x` completely to eliminate:
  - The hardcoded 50,000 us (20 Hz) _DEFAULT_REPORT_INTERVAL in adafruit_bno08x
  - Blocking timeout loops in _wait_for_packet()
  - Private method AttributeErrors across library versions

Publishes:
  - /imu/data_game (sensor_msgs/msg/Imu) @ 100 Hz (Game Rotation Vector 0x08 + Gyro 0x02)
  - /imu/data_mag  (sensor_msgs/msg/Imu) @ 50 Hz  (Rotation Vector 0x05 + Gyro 0x02)
"""

import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial


# SHTP Channels
CHANNEL_COMMAND = 0
CHANNEL_EXECUTABLE = 1
CHANNEL_CONTROL = 2
CHANNEL_REPORTS = 3

# SH-2 Control & Report IDs
SHTP_REPORT_PRODUCT_ID_REQ = 0xF9
SHTP_REPORT_SET_FEATURE_CMD = 0xFD
SHTP_REPORT_GET_FEATURE_RESP = 0xFC
SHTP_REPORT_BASE_TIMESTAMP = 0xFB

SENSOR_REPORTID_ACCELEROMETER = 0x01
SENSOR_REPORTID_GYROSCOPE = 0x02
SENSOR_REPORTID_ROTATION_VECTOR = 0x05       # With Magnetometer (/imu/data_mag)
SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08  # Accel + Gyro only (/imu/data_game)

# Fixed-point Q-scales per Hillcrest SH-2 Reference Manual
SCALE_QUAT = 1.0 / (1 << 14)   # Q14: 2^-14
SCALE_GYRO = 1.0 / (1 << 9)    # Q9:  2^-9 (rad/s)
SCALE_ACCEL = 1.0 / (1 << 8)   # Q8:  2^-8 (m/s^2)


class BNO085SHTPNode(Node):
    def __init__(self):
        super().__init__('bno085_shtp_node')

        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baudrate', 3000000)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('game_rate_hz', 100.0)
        self.declare_parameter('mag_rate_hz', 50.0)

        self.port = self.get_parameter('port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.game_rate_hz = float(self.get_parameter('game_rate_hz').value)
        self.mag_rate_hz = float(self.get_parameter('mag_rate_hz').value)

        self.game_interval_us = int(1_000_000 / self.game_rate_hz)  # 10,000 us (100 Hz)
        self.mag_interval_us = int(1_000_000 / self.mag_rate_hz)    # 20,000 us (50 Hz)

        self.pub_game = self.create_publisher(Imu, '/imu/data_game', 10)
        self.pub_mag = self.create_publisher(Imu, '/imu/data_mag', 10)

        # SHTP sequence number per channel (0..5)
        self.seq_num = [0, 0, 0, 0, 0, 0]
        self.rx_buf = bytearray()

        # Latest gyro & linear accel values to attach to Imu messages
        self.gyro_xyz = (0.0, 0.0, 0.0)
        self.accel_xyz = (0.0, 0.0, 0.0)

        self.uart = None
        self.running = True
        self.thread = threading.Thread(target=self.run_shtp_loop, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------------
    # Low-Level SHTP Packet Transmission
    # ------------------------------------------------------------------
    def send_shtp_packet(self, channel: int, payload: bytes):
        """Wrap payload in a 4-byte SHTP header and write to UART."""
        pkt_len = len(payload) + 4
        seq = self.seq_num[channel]
        self.seq_num[channel] = (seq + 1) & 0xFF

        header = struct.pack('<BBBB', pkt_len & 0xFF, (pkt_len >> 8) & 0x7F, channel, seq)
        # BNO08x UART-SHTP wakes/syncs best with a single 0x7E framing/wake byte or direct write
        self.uart.write(header + payload)
        self.uart.flush()

    def enable_sensor_feature(self, report_id: int, interval_us: int):
        """
        Send SH-2 Set Feature Command (0xFD) on Channel 2 (Control).
        17 bytes total:
          Byte 0:     0xFD (Set Feature Command)
          Byte 1:     Feature Report ID
          Byte 2:     Feature Flags (0)
          Bytes 3-4:  Change Sensitivity (0)
          Bytes 5-8:  Report Interval in microseconds (uint32 little-endian)
          Bytes 9-12: Batch Interval in microseconds (0 = no batching)
          Bytes 13-16: Sensor-specific config word (0)
        """
        cmd = struct.pack(
            '<BBBHIII',
            SHTP_REPORT_SET_FEATURE_CMD,
            report_id,
            0,
            0,
            interval_us,
            0,
            0,
        )
        self.send_shtp_packet(CHANNEL_CONTROL, cmd)

    def configure_bno085(self) -> bool:
        try:
            if self.uart is not None and self.uart.is_open:
                self.uart.close()

            self.uart = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.005,
                write_timeout=0.1,
            )
            self.uart.reset_input_buffer()
            self.uart.reset_output_buffer()
            self.rx_buf.clear()
            self.seq_num = [0, 0, 0, 0, 0, 0]

            self.get_logger().info(f'Soft-resetting BNO085 on {self.port} @ {self.baudrate} baud...')

            # 1. Send Executable Channel (1) Reset Command (byte = 1)
            self.send_shtp_packet(CHANNEL_EXECUTABLE, b'\x01')
            time.sleep(0.35)

            # Drain startup advertisement flood
            while self.uart.in_waiting > 0:
                self.uart.read(self.uart.in_waiting)
                time.sleep(0.02)

            # 2. Request Product ID (0xF9, 0x00) on Channel 2 to verify SH-2 is awake
            self.send_shtp_packet(CHANNEL_CONTROL, b'\xF9\x00')
            time.sleep(0.05)

            # 3. Enable features at target rates (send twice to guarantee reception)
            for _ in range(2):
                self.enable_sensor_feature(SENSOR_REPORTID_GAME_ROTATION_VECTOR, self.game_interval_us)
                time.sleep(0.02)
                self.enable_sensor_feature(SENSOR_REPORTID_ROTATION_VECTOR, self.mag_interval_us)
                time.sleep(0.02)
                self.enable_sensor_feature(SENSOR_REPORTID_GYROSCOPE, self.game_interval_us)
                time.sleep(0.02)

            self.rx_buf.clear()
            self.get_logger().info(
                f'BNO085 SHTP active! Target rates: /imu/data_game @ {self.game_rate_hz:.0f} Hz '
                f'({self.game_interval_us} us), /imu/data_mag @ {self.mag_rate_hz:.0f} Hz '
                f'({self.mag_interval_us} us).'
            )
            return True

        except Exception as e:
            self.get_logger().error(f'Failed to configure BNO085: {e}')
            return False

    # ------------------------------------------------------------------
    # Fast Binary SHTP Stream Parser
    # ------------------------------------------------------------------
    def parse_sensor_reports(self, payload: memoryview):
        """
        Parse Channel 3 (Input Sensor Reports) payload.
        A single SHTP packet on Channel 3 usually begins with 0xFB (Base Timestamp
        Reference, 5 bytes) followed by one or more concatenated sensor reports.
        """
        idx = 0
        n = len(payload)

        # Skip 5-byte base delta timestamp (0xFB) if present
        if n >= 5 and payload[0] == SHTP_REPORT_BASE_TIMESTAMP:
            idx = 5

        stamp = None

        while idx + 4 <= n:
            report_id = payload[idx]

            # 0x08: Game Rotation Vector (12 bytes total)
            # [0]=0x08, [1]=seq, [2]=status, [3]=delay, [4..11]=i, j, k, real (int16 x 4)
            if report_id == SENSOR_REPORTID_GAME_ROTATION_VECTOR:
                if idx + 12 > n:
                    break
                qi, qj, qk, qr = struct.unpack_from('<hhhh', payload, idx + 4)
                quat = (qi * SCALE_QUAT, qj * SCALE_QUAT, qk * SCALE_QUAT, qr * SCALE_QUAT)
                if stamp is None:
                    stamp = self.get_clock().now().to_msg()
                self.pub_game.publish(self.build_imu_msg(quat, stamp))
                idx += 12

            # 0x05: Rotation Vector with Magnetometer (14 bytes total)
            # [0]=0x05, [1]=seq, [2]=status, [3]=delay, [4..11]=i, j, k, real, [12..13]=accuracy
            elif report_id == SENSOR_REPORTID_ROTATION_VECTOR:
                if idx + 14 > n:
                    break
                qi, qj, qk, qr = struct.unpack_from('<hhhh', payload, idx + 4)
                quat = (qi * SCALE_QUAT, qj * SCALE_QUAT, qk * SCALE_QUAT, qr * SCALE_QUAT)
                if stamp is None:
                    stamp = self.get_clock().now().to_msg()
                self.pub_mag.publish(self.build_imu_msg(quat, stamp))
                idx += 14

            # 0x02: Calibrated Gyroscope (10 bytes total)
            # [0]=0x02, [1]=seq, [2]=status, [3]=delay, [4..9]=gx, gy, gz (int16 x 3)
            elif report_id == SENSOR_REPORTID_GYROSCOPE:
                if idx + 10 > n:
                    break
                gx, gy, gz = struct.unpack_from('<hhh', payload, idx + 4)
                self.gyro_xyz = (gx * SCALE_GYRO, gy * SCALE_GYRO, gz * SCALE_GYRO)
                idx += 10

            # 0x01: Accelerometer (10 bytes total)
            elif report_id == SENSOR_REPORTID_ACCELEROMETER:
                if idx + 10 > n:
                    break
                ax, ay, az = struct.unpack_from('<hhh', payload, idx + 4)
                self.accel_xyz = (ax * SCALE_ACCEL, ay * SCALE_ACCEL, az * SCALE_ACCEL)
                idx += 10

            # 0xFB: Inline timestamp reference (5 bytes)
            elif report_id == SHTP_REPORT_BASE_TIMESTAMP:
                idx += 5

            else:
                # Unknown report ID in this packet; stop parsing rest of this packet
                break

    def process_rx_buffer(self):
        """Extract complete SHTP packets from self.rx_buf."""
        while len(self.rx_buf) >= 4:
            len_lsb = self.rx_buf[0]
            len_msb = self.rx_buf[1]
            channel = self.rx_buf[2]

            # Clear continuation bit (bit 15) to get total packet length
            pkt_len = ((len_msb & 0x7F) << 8) | len_lsb

            # Sanity check on SHTP header: valid channels are 0..5, length 4..1024
            if pkt_len < 4 or pkt_len > 1024 or channel > 5:
                # Lost framing alignment: drop 1 byte and resync
                del self.rx_buf[0]
                continue

            # Wait until all bytes of this packet have arrived
            if len(self.rx_buf) < pkt_len:
                break

            # Extract packet payload (bytes 4 .. pkt_len)
            if channel == CHANNEL_REPORTS:
                view = memoryview(self.rx_buf)[4:pkt_len]
                self.parse_sensor_reports(view)
                del view
            elif channel == CHANNEL_CONTROL and pkt_len >= 13:
                # Log Get Feature Response (0xFC) so you can see the confirmed hardware rate!
                if self.rx_buf[4] == SHTP_REPORT_GET_FEATURE_RESP:
                    feat_id = self.rx_buf[5]
                    confirmed_us = struct.unpack_from('<I', self.rx_buf, 9)[0]
                    hz = 1_000_000.0 / confirmed_us if confirmed_us > 0 else 0.0
                    self.get_logger().info(
                        f'BNO085 Hardware Confirmed Feature 0x{feat_id:02X}: '
                        f'interval = {confirmed_us} us ({hz:.1f} Hz)'
                    )

            # Remove processed packet from sliding buffer
            del self.rx_buf[:pkt_len]

    def build_imu_msg(self, quat, stamp) -> Imu:
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        msg.orientation_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]

        msg.angular_velocity.x = self.gyro_xyz[0]
        msg.angular_velocity.y = self.gyro_xyz[1]
        msg.angular_velocity.z = self.gyro_xyz[2]
        msg.angular_velocity_covariance = [
            0.01, 0.0,  0.0,
            0.0,  0.01, 0.0,
            0.0,  0.0,  0.01,
        ]

        msg.linear_acceleration_covariance[0] = -1.0
        return msg

    def run_shtp_loop(self):
        if not self.configure_bno085():
            time.sleep(1.0)

        last_rx_time = time.monotonic()
        reconfig_timer = time.monotonic()

        while self.running and rclpy.ok():
            if self.uart is None or not self.uart.is_open:
                if self.configure_bno085():
                    last_rx_time = time.monotonic()
                else:
                    time.sleep(1.0)
                continue

            try:
                waiting = self.uart.in_waiting
                if waiting > 0:
                    chunk = self.uart.read(waiting)
                    if chunk:
                        self.rx_buf.extend(chunk)
                        self.process_rx_buffer()
                        last_rx_time = time.monotonic()
                else:
                    time.sleep(0.0005)  # 0.5 ms sleep when idle

                # Watchdog: if BNO085 resets itself due to a brownout or wire jiggle,
                # it stops sending reports until Set-Feature (0xFD) is sent again.
                now = time.monotonic()
                if now - last_rx_time > 0.5:
                    self.get_logger().warn('No BNO085 reports for 0.5s — re-sending feature enables...')
                    self.enable_sensor_feature(SENSOR_REPORTID_GAME_ROTATION_VECTOR, self.game_interval_us)
                    self.enable_sensor_feature(SENSOR_REPORTID_ROTATION_VECTOR, self.mag_interval_us)
                    self.enable_sensor_feature(SENSOR_REPORTID_GYROSCOPE, self.game_interval_us)
                    last_rx_time = now

            except Exception as e:
                self.get_logger().error(f'Serial exception: {e}. Reopening port...')
                self.configure_bno085()
                time.sleep(0.5)

    def destroy_node(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.uart is not None and self.uart.is_open:
            self.uart.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BNO085SHTPNode()
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
