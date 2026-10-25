// Copyright 2026 Agrobot

#include "agrobot_hardware/esp32_system_interface.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace agrobot_hardware
{

namespace
{

// Wire order of the ESP32 packet: FL, FR, RL, RR.
constexpr std::array<const char *, kNumWheels> kWheelJointNames = {
  "front_left_wheel_joint", "front_right_wheel_joint",
  "rear_left_wheel_joint", "rear_right_wheel_joint"};

constexpr std::size_t kMaxRxBuffer = kPacketSize * 16;
constexpr int kMaxDrainIterations = 64;
constexpr float kGainEpsilon = 1e-6f;

// Parses a gain from a URDF/xacro string. Rejects empty, non-numeric, non-finite
// and negative values.
bool parse_gain(const std::string & text, float & out)
{
  if (text.empty()) {
    return false;
  }
  try {
    const float v = std::stof(text);
    if (!std::isfinite(v) || v < 0.0f) {
      return false;
    }
    out = v;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

// NaN (no controller command yet) keeps the previous valid value; negatives clamp to 0.
float sanitize_gain(double raw, float fallback)
{
  const auto v = static_cast<float>(raw);
  return std::isfinite(v) ? std::max(v, 0.0f) : fallback;
}

bool gains_differ(const Gains & a, const Gains & b)
{
  return std::fabs(a.kp - b.kp) > kGainEpsilon || std::fabs(a.ki - b.ki) > kGainEpsilon ||
         std::fabs(a.kd - b.kd) > kGainEpsilon;
}

// Fletcher-16 over everything between the 0xAA 0x55 header and the checksum field
// (type byte + payload) of a host -> ESP32 packet.
template<typename Packet>
uint16_t tx_checksum(const Packet & pkt)
{
  return fletcher16(
    reinterpret_cast<const uint8_t *>(&pkt) + kTxBodyOffset,
    sizeof(Packet) - kTxBodyOffset - sizeof(uint16_t));
}

// Returns 0 (B0) if the baud rate is not supported.
speed_t baud_to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    case 2500000: return B2500000;
    case 3000000: return B3000000;
    case 3500000: return B3500000;
    case 4000000: return B4000000;
    default: return B0;
  }
}

}  // namespace

uint16_t fletcher16(const uint8_t * data, std::size_t len)
{
  uint32_t sum1 = 0;
  uint32_t sum2 = 0;
  for (std::size_t i = 0; i < len; ++i) {
    sum1 = (sum1 + data[i]) % 255;
    sum2 = (sum2 + sum1) % 255;
  }
  return static_cast<uint16_t>((sum2 << 8) | sum1);
}

ESP32SystemInterface::~ESP32SystemInterface()
{
  close_port();
}

// ============================================================================
// Initialisation / interface export
// ============================================================================

hardware_interface::CallbackReturn ESP32SystemInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- hardware parameters --------------------------------------------------
  const auto & params = info_.hardware_parameters;
  auto get_int = [&](const std::string & key, int & dst) -> bool {
      const auto it = params.find(key);
      if (it == params.end()) {
        return true;  // optional: keep default
      }
      try {
        dst = std::stoi(it->second);
      } catch (const std::exception &) {
        RCLCPP_FATAL(logger_, "Parameter '%s' is not an integer: '%s'", key.c_str(),
          it->second.c_str());
        return false;
      }
      return true;
    };

  const auto port_it = params.find("serial_port");
  if (port_it == params.end() || port_it->second.empty()) {
    RCLCPP_FATAL(logger_, "Hardware parameter 'serial_port' is required.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  serial_port_ = port_it->second;

  int comm_timeout_ms = static_cast<int>(comm_timeout_.count());
  int startup_timeout_ms = static_cast<int>(startup_timeout_.count());
  if (
    !get_int("baud_rate", baud_rate_) || !get_int("comm_timeout_ms", comm_timeout_ms) ||
    !get_int("startup_timeout_ms", startup_timeout_ms))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  comm_timeout_ = std::chrono::milliseconds(std::max(comm_timeout_ms, 50));
  startup_timeout_ = std::chrono::milliseconds(std::max(startup_timeout_ms, 100));

  if (baud_to_speed(baud_rate_) == B0) {
    RCLCPP_FATAL(logger_, "Unsupported baud_rate: %d", baud_rate_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---- joint validation + mapping to wire slots -----------------------------
  if (info_.joints.size() != kNumWheels) {
    RCLCPP_FATAL(logger_, "Expected %zu joints, URDF has %zu.", kNumWheels, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::array<bool, kNumWheels> slot_used{};
  for (std::size_t j = 0; j < info_.joints.size(); ++j) {
    const auto & joint = info_.joints[j];

    const auto slot_it =
      std::find_if(kWheelJointNames.begin(), kWheelJointNames.end(),
        [&](const char * n) {return joint.name == n;});
    if (slot_it == kWheelJointNames.end()) {
      RCLCPP_FATAL(logger_, "Unexpected joint '%s'.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const auto slot = static_cast<std::size_t>(slot_it - kWheelJointNames.begin());
    if (slot_used[slot]) {
      RCLCPP_FATAL(logger_, "Joint '%s' defined twice.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    slot_used[slot] = true;
    joint_to_slot_[j] = slot;

    // Command interfaces: velocity + kp/ki/kd (any order). The gains must carry an
    // initial_value so the ESP32 is never handed a guessed PID configuration.
    bool has_vel_cmd = false;
    std::array<bool, 3> has_gain{};
    for (const auto & ci : joint.command_interfaces) {
      if (ci.name == hardware_interface::HW_IF_VELOCITY) {
        has_vel_cmd = true;
        continue;
      }
      std::size_t idx = 0;
      float * target = nullptr;
      if (ci.name == kIfKp) {
        idx = 0;
        target = &initial_gains_[slot].kp;
      } else if (ci.name == kIfKi) {
        idx = 1;
        target = &initial_gains_[slot].ki;
      } else if (ci.name == kIfKd) {
        idx = 2;
        target = &initial_gains_[slot].kd;
      } else {
        RCLCPP_FATAL(logger_, "Joint '%s': unexpected command interface '%s'.",
          joint.name.c_str(), ci.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      if (!parse_gain(ci.initial_value, *target)) {
        RCLCPP_FATAL(logger_,
          "Joint '%s': command interface '%s' needs a numeric initial_value >= 0 (got '%s').",
          joint.name.c_str(), ci.name.c_str(), ci.initial_value.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
      has_gain[idx] = true;
    }
    if (
      joint.command_interfaces.size() != 4 || !has_vel_cmd || !has_gain[0] || !has_gain[1] ||
      !has_gain[2])
    {
      RCLCPP_FATAL(logger_,
        "Joint '%s' needs exactly four command interfaces: velocity, kp, ki, kd.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_vel = false;
    bool has_pos = false;
    for (const auto & si : joint.state_interfaces) {
      has_vel |= (si.name == hardware_interface::HW_IF_VELOCITY);
      has_pos |= (si.name == hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces.size() != 2 || !has_vel || !has_pos) {
      RCLCPP_FATAL(logger_, "Joint '%s' needs 'velocity' and 'position' state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hw_commands_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_positions_.fill(0.0);
  reset_gain_state();

  RCLCPP_INFO(logger_, "Initialised: port=%s baud=%d", serial_port_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ESP32SystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(2 * kNumWheels);
  for (std::size_t j = 0; j < info_.joints.size(); ++j) {
    const auto slot = joint_to_slot_[j];
    interfaces.emplace_back(
      info_.joints[j].name, hardware_interface::HW_IF_POSITION, &hw_positions_[slot]);
    interfaces.emplace_back(
      info_.joints[j].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[slot]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> ESP32SystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(4 * kNumWheels);
  for (std::size_t j = 0; j < info_.joints.size(); ++j) {
    const auto slot = joint_to_slot_[j];
    const auto & name = info_.joints[j].name;
    interfaces.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[slot]);
    interfaces.emplace_back(name, kIfKp, &hw_kp_[slot]);
    interfaces.emplace_back(name, kIfKi, &hw_ki_[slot]);
    interfaces.emplace_back(name, kIfKd, &hw_kd_[slot]);
  }
  return interfaces;
}

// ============================================================================
// Lifecycle
// ============================================================================

hardware_interface::CallbackReturn ESP32SystemInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  close_port();
  if (!open_port()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_commands_.fill(0.0);
  hw_velocities_.fill(0.0);
  hw_positions_.fill(0.0);
  reset_gain_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (fd_ < 0 && !open_port()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  tcflush(fd_, TCIFLUSH);
  rx_buffer_.clear();

  hw_commands_.fill(0.0);
  hw_velocities_.fill(0.0);

  invalidate_gain_cache();
  have_received_ = false;
  last_rx_time_ = SteadyClock::now();
  packets_ok_ = bad_checksum_count_ = dropped_byte_count_ = 0;
  reported_bad_checksum_ = reported_dropped_bytes_ = 0;

  send_zero_velocity();
  RCLCPP_INFO(logger_, "Activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  hw_commands_.fill(0.0);
  send_zero_velocity();
  RCLCPP_INFO(logger_, "Deactivated (wheels commanded to zero). OK packets: %lu",
    static_cast<unsigned long>(packets_ok_));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_shutdown(const rclcpp_lifecycle::State &)
{
  send_zero_velocity();
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ESP32SystemInterface::on_error(const rclcpp_lifecycle::State &)
{
  hw_commands_.fill(0.0);
  send_zero_velocity();  // best effort; ignored if the link is already gone
  close_port();
  return hardware_interface::CallbackReturn::SUCCESS;  // -> unconfigured, can be re-configured
}

// ============================================================================
// read() / write()
// ============================================================================

hardware_interface::return_type ESP32SystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now = SteadyClock::now();

  // Port lost (USB unplugged, I/O error)? Try to re-open at a limited rate.
  if (fd_ < 0 && now - last_reconnect_attempt_ >= reconnect_period_) {
    last_reconnect_attempt_ = now;
    if (open_port()) {
      RCLCPP_INFO(logger_, "Serial port re-opened.");
      have_received_ = false;  // ESP32 may have reset on open: use startup timeout
      last_rx_time_ = now;
    }
  }

  std::array<float, kNumWheels> latest{};
  bool got_packet = false;
  if (fd_ >= 0) {
    drain_serial();
    got_packet = extract_latest_packet(latest);
  }

  if (got_packet) {
    if (!have_received_) {
      // First packet after start-up or a comm gap: the ESP32 may have rebooted and
      // lost its PID gains, so push every wheel's gains again.
      invalidate_gain_cache();
    }
    // Integrate position from the wheel velocities reported by the ESP32.
    double dt = 0.0;
    if (have_received_) {
      dt = std::min(std::chrono::duration<double>(now - last_rx_time_).count(), 0.1);
    }
    for (std::size_t i = 0; i < kNumWheels; ++i) {
      hw_velocities_[i] = static_cast<double>(latest[i]);
      hw_positions_[i] += hw_velocities_[i] * dt;
    }
    last_rx_time_ = now;
    have_received_ = true;
    ++packets_ok_;
  } else {
    const auto timeout = have_received_ ? comm_timeout_ : startup_timeout_;
    if (now - last_rx_time_ > timeout) {
      RCLCPP_ERROR_THROTTLE(
        logger_, steady_clock_, 1000,
        "No valid packet from ESP32 for > %ld ms (port %s, %s). Reporting ERROR.",
        static_cast<long>(timeout.count()), serial_port_.c_str(),
        fd_ < 0 ? "disconnected" : "connected");
      return hardware_interface::return_type::ERROR;
    }
  }

  if (
    bad_checksum_count_ != reported_bad_checksum_ || dropped_byte_count_ != reported_dropped_bytes_)
  {
    RCLCPP_WARN_THROTTLE(
      logger_, steady_clock_, 2000,
      "Serial re-sync activity: %lu bad checksums, %lu bytes discarded (totals).",
      static_cast<unsigned long>(bad_checksum_count_),
      static_cast<unsigned long>(dropped_byte_count_));
    reported_bad_checksum_ = bad_checksum_count_;
    reported_dropped_bytes_ = dropped_byte_count_;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ESP32SystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (fd_ < 0) {
    return hardware_interface::return_type::OK;  // read() handles reconnect / timeout
  }

  // Type 0x02: only for wheels whose gains changed since the last cycle.
  send_changed_gains();

  // Type 0x01: always, every update cycle (50 Hz).
  std::array<float, kNumWheels> cmd{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const auto v = static_cast<float>(hw_commands_[i]);
    cmd[i] = std::isfinite(v) ? v : 0.0f;  // NaN before first controller update
  }

  if (!send_packet(cmd)) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 1000, "Failed to write command packet.");
  }
  return hardware_interface::return_type::OK;
}

// ============================================================================
// Serial helpers
// ============================================================================

bool ESP32SystemInterface::open_port()
{
  close_port();

  const int fd = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    RCLCPP_ERROR_THROTTLE(
      logger_, steady_clock_, 2000, "Cannot open %s: %s", serial_port_.c_str(),
      std::strerror(errno));
    return false;
  }

  ioctl(fd, TIOCEXCL);  // exclusive access: no other process may open the port

  termios tty{};
  if (tcgetattr(fd, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcgetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }

  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tty.c_cflag |= CS8;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  const speed_t speed = baud_to_speed(baud_rate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(logger_, "tcsetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }

  // De-assert DTR/RTS (auto-reset circuit on many ESP32 dev boards).
  int modem_bits = TIOCM_DTR | TIOCM_RTS;
  ioctl(fd, TIOCMBIC, &modem_bits);

  tcflush(fd, TCIOFLUSH);
  fd_ = fd;
  rx_buffer_.clear();
  RCLCPP_INFO(logger_, "Opened %s @ %d baud (8N1).", serial_port_.c_str(), baud_rate_);
  return true;
}

void ESP32SystemInterface::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rx_buffer_.clear();
}

void ESP32SystemInterface::drain_serial()
{
  uint8_t tmp[256];
  for (int iter = 0; iter < kMaxDrainIterations; ++iter) {
    const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0) {
      rx_buffer_.insert(rx_buffer_.end(), tmp, tmp + n);
      continue;
    }
    if (n == 0) {
      break;  // nothing pending
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    RCLCPP_ERROR(logger_, "Serial read failed (%s). Closing port, will retry.",
      std::strerror(errno));
    close_port();
    return;
  }

  // Keep only the newest bytes if we fell far behind.
  if (rx_buffer_.size() > kMaxRxBuffer) {
    const std::size_t excess = rx_buffer_.size() - kMaxRxBuffer;
    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(excess));
    dropped_byte_count_ += excess;
  }
}

bool ESP32SystemInterface::extract_latest_packet(std::array<float, kNumWheels> & out)
{
  bool got = false;
  std::size_t pos = 0;

  while (rx_buffer_.size() - pos >= kPacketSize) {
    // Re-sync: skip until the 0xAA 0x55 header.
    if (rx_buffer_[pos] != kHeader0 || rx_buffer_[pos + 1] != kHeader1) {
      ++pos;
      ++dropped_byte_count_;
      continue;
    }

    SerialPacket pkt{};
    std::memcpy(&pkt, rx_buffer_.data() + pos, kPacketSize);

    const uint16_t expected = fletcher16(rx_buffer_.data() + pos + kPayloadOffset, kPayloadSize);
    bool valid = (pkt.checksum == expected);
    if (valid) {
      for (std::size_t i = 0; i < kNumWheels; ++i) {
        const float v = pkt.velocities[i];
        if (!std::isfinite(v)) {
          valid = false;
          break;
        }
        out[i] = v;
      }
    }

    if (valid) {
      got = true;  // keep scanning: the newest valid packet wins
      pos += kPacketSize;
    } else {
      // False header (0xAA 0x55 inside payload) or corrupted frame: slip one byte.
      ++bad_checksum_count_;
      ++pos;
      ++dropped_byte_count_;
    }
  }

  // Discard leading garbage in the (< 1 packet) tail, keep a possible partial header.
  while (pos < rx_buffer_.size() && rx_buffer_[pos] != kHeader0) {
    ++pos;
    ++dropped_byte_count_;
  }

  rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(pos));
  return got;
}

void ESP32SystemInterface::reset_gain_state()
{
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    hw_kp_[i] = initial_gains_[i].kp;
    hw_ki_[i] = initial_gains_[i].ki;
    hw_kd_[i] = initial_gains_[i].kd;
    active_gains_[i] = initial_gains_[i];
    sent_gains_[i] = Gains{};
  }
  invalidate_gain_cache();
}

void ESP32SystemInterface::invalidate_gain_cache()
{
  gains_in_sync_.fill(false);
}

void ESP32SystemInterface::send_changed_gains()
{
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    const Gains candidate{
      sanitize_gain(hw_kp_[i], active_gains_[i].kp),
      sanitize_gain(hw_ki_[i], active_gains_[i].ki),
      sanitize_gain(hw_kd_[i], active_gains_[i].kd)};
    active_gains_[i] = candidate;

    if (gains_in_sync_[i] && !gains_differ(candidate, sent_gains_[i])) {
      continue;  // unchanged since the last cycle: nothing to send
    }

    if (!send_gain_packet(i, candidate)) {
      gains_in_sync_[i] = false;  // retry next cycle
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, 1000, "Failed to write gain packet for motor %zu.", i);
      return;  // link is likely down; don't hammer it with the other wheels
    }
    sent_gains_[i] = candidate;
    gains_in_sync_[i] = true;
    RCLCPP_DEBUG(logger_, "Motor %zu gains -> kp=%.4f ki=%.4f kd=%.4f", i,
      static_cast<double>(candidate.kp), static_cast<double>(candidate.ki),
      static_cast<double>(candidate.kd));
  }
}

// Type 0x01
bool ESP32SystemInterface::send_packet(const std::array<float, kNumWheels> & velocities)
{
  VelocityTxPacket pkt{};
  for (std::size_t i = 0; i < kNumWheels; ++i) {
    pkt.velocities[i] = velocities[i];
  }
  pkt.checksum = tx_checksum(pkt);
  return write_all(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
}

// Type 0x02
bool ESP32SystemInterface::send_gain_packet(std::size_t slot, const Gains & gains)
{
  GainTxPacket pkt{};
  pkt.motor_id = static_cast<uint8_t>(slot);
  pkt.kp = gains.kp;
  pkt.ki = gains.ki;
  pkt.kd = gains.kd;
  pkt.checksum = tx_checksum(pkt);
  return write_all(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));
}

bool ESP32SystemInterface::write_all(const uint8_t * raw, std::size_t len)
{
  if (fd_ < 0) {
    return false;
  }

  std::size_t sent = 0;
  while (sent < len) {
    const ssize_t n = ::write(fd_, raw + sent, len - sent);
    if (n > 0) {
      sent += static_cast<std::size_t>(n);
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd pfd{fd_, POLLOUT, 0};
      if (::poll(&pfd, 1, 5) <= 0) {
        return false;  // TX buffer stuck; ESP32 parser will re-sync on next frame
      }
      continue;
    }
    RCLCPP_ERROR(logger_, "Serial write failed (%s). Closing port, will retry.",
      std::strerror(errno));
    close_port();
    return false;
  }
  return true;
}

void ESP32SystemInterface::send_zero_velocity()
{
  const std::array<float, kNumWheels> zeros{};
  for (int i = 0; i < 2; ++i) {
    if (!send_packet(zeros)) {
      break;
    }
  }
}

}  // namespace agrobot_hardware

PLUGINLIB_EXPORT_CLASS(agrobot_hardware::ESP32SystemInterface, hardware_interface::SystemInterface)
