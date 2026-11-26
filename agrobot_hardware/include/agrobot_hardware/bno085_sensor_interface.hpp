// Copyright 2026 Agrobot
//
// ros2_control SensorInterface for a BNO085 IMU on a Linux I2C bus (no Python, no
// Blinka). A background thread talks SHTP over /dev/i2c-N and keeps the latest sample;
// read() just copies it, so a slow or failing I2C bus can never stall the control loop.
//
// URDF: one <ros2_control type="sensor"> with one <sensor> per fused output, each with
// the 10 imu_sensor_broadcaster interfaces (orientation.x/y/z/w, angular_velocity.x/y/z,
// linear_acceleration.x/y/z) and a <param name="source">:
//   game      Game Rotation Vector (gyro+accel only, no magnetometer, drift-free yaw rate)
//   absolute  Rotation Vector (adds the magnetometer: absolute heading)
// Gyro and accelerometer are shared by all sensors.
//
// Hardware params: i2c_bus (3), i2c_address (0x4A), reset_gpio (BCM line, 4; -1 = none),
// reset_gpio_chip ("auto" or e.g. "/dev/gpiochip4"), game_rate_hz (100), mag_rate_hz (5),
// stale_timeout_ms (250). A field older than the timeout (>= 3 report periods) reads NaN.

#ifndef AGROBOT_HARDWARE__BNO085_SENSOR_INTERFACE_HPP_
#define AGROBOT_HARDWARE__BNO085_SENSOR_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "agrobot_hardware/bno085_shtp.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace agrobot_hardware
{

class BNO085SensorInterface : public hardware_interface::SensorInterface
{
public:
  BNO085SensorInterface() = default;
  ~BNO085SensorInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;

  // ---- reader thread only ---------------------------------------------------
  void reader_loop();
  bool init_sensor();
  bool enable_features();
  bool open_i2c();
  void close_i2c();
  bool pulse_reset();
  void release_reset();
  bool send_packet(uint8_t channel, const std::vector<uint8_t> & payload);
  // 1 = packet read, 0 = nothing pending, -1 = I2C error
  int read_packet(shtp::Header & hdr, std::vector<uint8_t> & payload);
  void handle_packet(const shtp::Header & hdr, const std::vector<uint8_t> & payload);
  static std::string find_gpio_chip(int line);

  rclcpp::Logger logger_ = rclcpp::get_logger("BNO085SensorInterface");
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  // ---- parameters -----------------------------------------------------------
  int i2c_bus_{3};
  int i2c_address_{0x4A};
  int reset_gpio_{4};
  std::string reset_chip_{"auto"};
  double game_rate_hz_{100.0};
  double mag_rate_hz_{5.0};
  int stale_timeout_ms_{250};

  // ---- device (reader thread) -------------------------------------------------
  int i2c_fd_{-1};
  int reset_fd_{-1};
  std::array<uint8_t, 8> seq_{};
  std::vector<uint8_t> rx_;
  std::atomic<bool> need_reenable_{false};
  bool initialised_{false};

  // ---- shared state -----------------------------------------------------------
  struct Latest
  {
    bool seen_accel{false}, seen_gyro{false}, seen_game{false}, seen_rotation{false};
    double accel[3]{}, gyro[3]{}, game[4]{}, rotation[4]{};
    Clock::time_point t_accel{}, t_gyro{}, t_game{}, t_rotation{};
  };
  std::mutex mutex_;
  Latest latest_;
  std::thread thread_;
  std::atomic<bool> running_{false};

  // ---- exported interfaces (index = <sensor> order in the URDF) ----------------
  static constexpr std::size_t kNumValues = 10;
  std::vector<std::array<double, kNumValues>> values_;
  std::vector<bool> absolute_;  // true: Rotation Vector, false: Game Rotation Vector
};

}  // namespace agrobot_hardware

#endif  // AGROBOT_HARDWARE__BNO085_SENSOR_INTERFACE_HPP_
