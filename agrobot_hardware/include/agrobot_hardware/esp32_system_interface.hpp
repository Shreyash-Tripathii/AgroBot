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
// ESP32 -> host telemetry is unchanged: 20 B, AA 55 | 4 x float rad/s | fletcher16
// (checksum over the 16 payload bytes only).

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
  std::array<std::size_t, kNumWheels> joint_to_slot_{};  // info_.joints index -> slot

  // --- gain cache (indexed by wire slot) ------------------------------------
  std::array<Gains, kNumWheels> initial_gains_{};  // from the URDF initial_value params
  std::array<Gains, kNumWheels> active_gains_{};   // last valid (finite, >= 0) request
  std::array<Gains, kNumWheels> sent_gains_{};     // last gains successfully written
  std::array<bool, kNumWheels> gains_in_sync_{};   // false => (re)send on next write()
};

}  // namespace agrobot_hardware

#endif  // AGROBOT_HARDWARE__ESP32_SYSTEM_INTERFACE_HPP_
