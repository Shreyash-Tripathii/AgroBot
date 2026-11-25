// Copyright 2026 Agrobot
//
// ros2_control SystemInterface for the Agrobot 4WD base. Talks to an ESP32
// (50 Hz wheel PID) over a USB serial link.
//
// Host -> ESP32 (packed, little-endian). Both packet types start with 0xAA 0x55.
// The Fletcher-16 checksum covers every byte between the header and the checksum
// field (so it includes the type byte):
//
//   Type 0x01 velocity (21 B): AA 55 | 01 | 4 x float rad/s (FL,FR,RL,RR) | fletcher16
//   Type 0x02 gains    (18 B): AA 55 | 02 | motor_id u8 (0=FL,1=FR,2=RL,3=RR)
//                              | kp f32 | ki f32 | kd f32 | fletcher16
//
//   Type 0x03 servos   (11 B): AA 55 | 03 | 3 x uint16 pulse width in us (servo 0,1,2,
//                              0 = output off / servo limp) | fletcher16
//
// ESP32 -> host telemetry is unchanged: 20 B, AA 55 | 4 x float rad/s | fletcher16
// (checksum over the 16 payload bytes only). The MG995 arm servos are open loop, so
// there is no servo feedback: the reported joint state is the host's slew-limited
// estimate of the servo position.
//
// ESP32 -> host also sends a 23 B diagnostics frame at 1 Hz (header AA 56, own checksum):
//   AA 56 | version u8 | uptime_ms u32 | watchdog_trips, rx_frames, rx_bad_checksum,
//   rx_skipped_bytes, max_pass_us, max_pid_period_us, pid_mutex_misses (7 x u16) | fletcher16
// version >= 1 means the firmware understands 0x03; the host sends servo frames only after
// it has seen such a frame (old firmware would choke on them), and then only when a pulse
// changes plus a 5 Hz keep-alive.
//
// The ESP32 runs a link watchdog: if no valid packet arrives for LINK_TIMEOUT_MS it
// zeroes all motor PWM and releases every servo until the host talks to it again.
// write() only runs while this hardware is ACTIVE, so deactivating/shutting down the
// host stops the servo keep-alive too: the arm is held for at most the watchdog timeout
// and then goes limp (it may sag under gravity). Park the arm in a safe pose first.

#ifndef AGROBOT_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_
#define AGROBOT_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace agrobot_hardware
{

constexpr std::size_t kNumWheels = 4;  // Order on the wire: FL, FR, RL, RR
constexpr std::size_t kNumServos = 3;  // Arm servo channels 0..2 (optional joints)
constexpr uint8_t kHeader0 = 0xAA;
constexpr uint8_t kHeader1 = 0x55;

// Custom command interface names (besides "velocity").
constexpr const char * kIfKp = "kp";
constexpr const char * kIfKi = "ki";
constexpr const char * kIfKd = "kd";

// ---- ESP32 -> host telemetry (unchanged) -----------------------------------
// 20 bytes total: Header (2) + 4x float (16) + Fletcher-16 checksum (2)
struct __attribute__((packed)) SerialPacket
{
  uint8_t header[2] = {kHeader0, kHeader1};
  float velocities[kNumWheels];  // rad/s, order: FL, FR, RL, RR
  uint16_t checksum;
};
static_assert(sizeof(SerialPacket) == 20, "SerialPacket must be exactly 20 bytes");

constexpr std::size_t kPacketSize = sizeof(SerialPacket);
constexpr std::size_t kPayloadOffset = 2;                         // first float
constexpr std::size_t kPayloadSize = sizeof(float) * kNumWheels;  // 16 bytes

// ---- host -> ESP32 ----------------------------------------------------------
constexpr uint8_t kTypeVelocity = 0x01;
constexpr uint8_t kTypeGains = 0x02;
constexpr uint8_t kTypeServo = 0x03;
constexpr std::size_t kTxBodyOffset = 2;  // checksum starts at the type byte

// 21 bytes: Header (2) + Type (1) + 4x float (16) + checksum (2)
struct __attribute__((packed)) VelocityTxPacket
{
  uint8_t header[2] = {kHeader0, kHeader1};
  uint8_t type = kTypeVelocity;
  float velocities[kNumWheels];  // rad/s, order: FL, FR, RL, RR
  uint16_t checksum;
};
static_assert(sizeof(VelocityTxPacket) == 21, "VelocityTxPacket must be exactly 21 bytes");

// 18 bytes: Header (2) + Type (1) + Motor ID (1) + 3x float (12) + checksum (2)
struct __attribute__((packed)) GainTxPacket
{
  uint8_t header[2] = {kHeader0, kHeader1};
  uint8_t type = kTypeGains;
  uint8_t motor_id;  // wire slot: 0=FL, 1=FR, 2=RL, 3=RR
  float kp;
  float ki;
  float kd;
  uint16_t checksum;
};
static_assert(sizeof(GainTxPacket) == 18, "GainTxPacket must be exactly 18 bytes");

// 11 bytes: Header (2) + Type (1) + 3x uint16 (6) + checksum (2)
struct __attribute__((packed)) ServoTxPacket
{
  uint8_t header[2] = {kHeader0, kHeader1};
  uint8_t type = kTypeServo;
  uint16_t pulse_us[kNumServos];  // 0 = off, otherwise ~500..2500
  uint16_t checksum;
};
static_assert(sizeof(ServoTxPacket) == 11, "ServoTxPacket must be exactly 11 bytes");

// ---- ESP32 -> host diagnostics (AA 56) ---------------------------------------
constexpr uint8_t kStatusHeader1 = 0x56;
constexpr uint8_t kStatusVersionServo = 1;
struct __attribute__((packed)) StatusPacket
{
  uint8_t header[2];
  uint8_t version;
  uint32_t uptime_ms;
  uint16_t watchdog_trips;
  uint16_t rx_frames;
  uint16_t rx_bad_checksum;
  uint16_t rx_skipped_bytes;
  uint16_t max_pass_us;
  uint16_t max_pid_period_us;
  uint16_t pid_mutex_misses;
  uint16_t checksum;
};
static_assert(sizeof(StatusPacket) == 23, "StatusPacket must be exactly 23 bytes");
constexpr std::size_t kStatusPacketSize = sizeof(StatusPacket);
constexpr std::size_t kStatusBodyOffset = 2;
constexpr std::size_t kStatusPayloadSize = kStatusPacketSize - 4;  // after header, before checksum

/// Per-servo-joint configuration, parsed from the URDF (joint params / interface limits).
struct ServoConfig
{
  double zero_pulse_us{1500.0};  // pulse at joint angle 0
  double us_per_rad{636.62};     // pulse change per rad; negative reverses direction
  double min_pulse_us{500.0};
  double max_pulse_us{2500.0};
  double max_velocity{1.5};      // rad/s slew limit (also the reported velocity bound)
  double min_pos{-1.5708};       // rad
  double max_pos{1.5708};        // rad
  double home{0.0};              // rad, position before the first command
};

struct Gains
{
  float kp{0.0f};
  float ki{0.0f};
  float kd{0.0f};
};

/// Fletcher-16 over `len` bytes. Result = (sum2 << 8) | sum1.
uint16_t fletcher16(const uint8_t * data, std::size_t len);

class ESP32SystemInterface : public hardware_interface::SystemInterface
{
public:
  ESP32SystemInterface() = default;
  ~ESP32SystemInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using SteadyClock = std::chrono::steady_clock;

  // --- serial helpers -------------------------------------------------------
  bool open_port();
  void close_port();
  void drain_serial();
  bool extract_latest_packet(std::array<float, kNumWheels> & out);
  bool write_all(const uint8_t * raw, std::size_t len);
  bool send_packet(const std::array<float, kNumWheels> & velocities);  // Type 0x01
  bool send_gain_packet(std::size_t slot, const Gains & gains);        // Type 0x02
  std::array<uint16_t, kNumServos> compute_servo_pulses() const;
  bool send_servo_packet(const std::array<uint16_t, kNumServos> & pulses);  // Type 0x03
  void handle_status(const StatusPacket & st);
  void log_timing_summary(SteadyClock::time_point now);
  void update_servo_setpoints(double dt);  // slew-limit command -> estimated position
  void send_zero_velocity();

  // --- gain handling --------------------------------------------------------
  void reset_gain_state();         // hw_k* <- URDF initial values, everything marked unsent
  void invalidate_gain_cache();    // force every wheel's gains to be re-sent
  void send_changed_gains();       // evaluate the cache, send Type 0x02 for changed wheels

  // --- logging --------------------------------------------------------------
  rclcpp::Logger logger_ = rclcpp::get_logger("ESP32SystemInterface");
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // for *_THROTTLE macros

  // --- parameters -----------------------------------------------------------
  std::string serial_port_{"/dev/ttyUSB0"};
  int baud_rate_{500000};
  std::chrono::milliseconds comm_timeout_{500};      // max silence once streaming
  std::chrono::milliseconds startup_timeout_{3000};  // max wait for first packet
  std::chrono::milliseconds reconnect_period_{500};

  // --- serial state ---------------------------------------------------------
  int fd_{-1};
  std::vector<uint8_t> rx_buffer_;
  SteadyClock::time_point last_rx_time_{};
  SteadyClock::time_point last_reconnect_attempt_{};
  bool have_received_{false};

  uint64_t packets_ok_{0};
  uint64_t bad_checksum_count_{0};
  uint64_t dropped_byte_count_{0};
  uint64_t reported_bad_checksum_{0};
  uint64_t reported_dropped_bytes_{0};

  // --- ros2_control data (indexed by wire slot: FL, FR, RL, RR) -------------
  std::array<double, kNumWheels> hw_commands_{};
  std::array<double, kNumWheels> hw_velocities_{};
  std::array<double, kNumWheels> hw_positions_{};
  std::array<double, kNumWheels> hw_kp_{};  // command interfaces "kp", "ki", "kd"
  std::array<double, kNumWheels> hw_ki_{};
  std::array<double, kNumWheels> hw_kd_{};
  // info_.joints index -> wheel slot (is_servo == false) or servo channel (true)
  struct JointMap
  {
    bool is_servo{false};
    std::size_t index{0};
  };
  std::vector<JointMap> joint_map_;

  // --- ESP32 status / capability ---------------------------------------------
  bool status_seen_{false};
  bool esp_has_servo_{false};
  StatusPacket last_status_{};
  SteadyClock::time_point status_wait_start_{};
  SteadyClock::time_point last_servo_tx_{};  // epoch => send on next write()
  std::array<uint16_t, kNumServos> last_servo_pulse_{};
  std::chrono::milliseconds servo_keepalive_{200};  // << ESP32 LINK_TIMEOUT_MS (500)

  // --- loop timing diagnostics (max over each 10 s window) -----------------------
  double max_read_us_{0.0};
  double max_write_us_{0.0};
  double max_write_period_us_{0.0};
  uint32_t servo_frames_window_{0};
  bool have_last_write_{false};
  SteadyClock::time_point last_write_time_{};
  SteadyClock::time_point last_summary_time_{};

  // --- arm servos (indexed by servo channel; all zero/false when no arm joints) ----
  std::array<ServoConfig, kNumServos> servo_cfg_{};
  std::array<bool, kNumServos> servo_configured_{};
  std::array<double, kNumServos> hw_servo_cmd_{};  // command interface "position"
  std::array<double, kNumServos> hw_servo_pos_{};  // state: slew-limited estimate
  std::array<double, kNumServos> hw_servo_vel_{};
  bool any_servo_{false};

  // --- gain cache (indexed by wire slot) ------------------------------------
  std::array<Gains, kNumWheels> initial_gains_{};  // from the URDF initial_value params
  std::array<Gains, kNumWheels> active_gains_{};   // last valid (finite, >= 0) request
  std::array<Gains, kNumWheels> sent_gains_{};     // last gains successfully written
  std::array<bool, kNumWheels> gains_in_sync_{};   // false => (re)send on next write()
};

}  // namespace agrobot_hardware

#endif  // AGROBOT_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_
