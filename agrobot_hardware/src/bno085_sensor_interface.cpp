// Copyright 2026 Agrobot

#include "agrobot_hardware/bno085_sensor_interface.hpp"

#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace agrobot_hardware
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr std::size_t kMaxPacket = 1024;

// Order matches semantic_components::IMUSensor / imu_sensor_broadcaster.
constexpr std::array<const char *, 10> kInterfaceNames = {
  "orientation.x", "orientation.y", "orientation.z", "orientation.w",
  "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
  "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"};

void sleep_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

}  // namespace

BNO085SensorInterface::~BNO085SensorInterface()
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  close_i2c();
  release_reset();
}

// ============================================================================
// Init / interface export
// ============================================================================

hardware_interface::CallbackReturn BNO085SensorInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SensorInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hp = info_.hardware_parameters;
  auto get_num = [&](const char * key, double & dst) -> bool {
      const auto it = hp.find(key);
      if (it == hp.end()) {
        return true;
      }
      try {
        dst = std::stod(it->second);
      } catch (const std::exception &) {
        RCLCPP_FATAL(logger_, "Hardware parameter '%s' is not a number: '%s'", key,
          it->second.c_str());
        return false;
      }
      return true;
    };

  double bus = i2c_bus_, reset = reset_gpio_, game = game_rate_hz_, mag = mag_rate_hz_;
  double stale = stale_timeout_ms_;
  if (!get_num("i2c_bus", bus) || !get_num("reset_gpio", reset) ||
    !get_num("game_rate_hz", game) || !get_num("mag_rate_hz", mag) ||
    !get_num("stale_timeout_ms", stale))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  i2c_bus_ = static_cast<int>(bus);
  reset_gpio_ = static_cast<int>(reset);
  game_rate_hz_ = game;
  mag_rate_hz_ = mag;
  stale_timeout_ms_ = std::max(static_cast<int>(stale), 50);

  if (const auto it = hp.find("i2c_address"); it != hp.end()) {
    try {
      i2c_address_ = std::stoi(it->second, nullptr, 0);  // base 0: "0x4A" or "74"
    } catch (const std::exception &) {
      RCLCPP_FATAL(logger_, "Hardware parameter 'i2c_address' is invalid: '%s'",
        it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  if (const auto it = hp.find("reset_gpio_chip"); it != hp.end() && !it->second.empty()) {
    reset_chip_ = it->second;
  }
  if (game_rate_hz_ < 1.0 || game_rate_hz_ > 400.0 || mag_rate_hz_ < 0.5 ||
    mag_rate_hz_ > 400.0)
  {
    RCLCPP_FATAL(logger_, "game_rate_hz / mag_rate_hz out of range (1..400 / 0.5..400).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.empty()) {
    RCLCPP_FATAL(logger_, "At least one <sensor> is required.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  values_.assign(info_.sensors.size(), {});
  absolute_.assign(info_.sensors.size(), false);
  for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
    const auto & s = info_.sensors[i];
    values_[i].fill(kNaN);
    const auto src = s.parameters.find("source");
    const std::string source = src == s.parameters.end() ? "game" : src->second;
    if (source != "game" && source != "absolute") {
      RCLCPP_FATAL(logger_, "Sensor '%s': param 'source' must be 'game' or 'absolute' (got '%s').",
        s.name.c_str(), source.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    absolute_[i] = (source == "absolute");
    for (const auto & si : s.state_interfaces) {
      const bool known = std::any_of(
        kInterfaceNames.begin(), kInterfaceNames.end(),
        [&](const char * n) {return si.name == n;});
      if (!known) {
        RCLCPP_FATAL(logger_, "Sensor '%s': unknown state interface '%s'.", s.name.c_str(),
          si.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  RCLCPP_INFO(logger_, "Initialised: /dev/i2c-%d addr 0x%02X, game %.0f Hz, rotation %.1f Hz, "
    "%zu sensor(s)", i2c_bus_, i2c_address_, game_rate_hz_, mag_rate_hz_, info_.sensors.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> BNO085SensorInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.sensors.size(); ++i) {
    for (const auto & si : info_.sensors[i].state_interfaces) {
      for (std::size_t k = 0; k < kInterfaceNames.size(); ++k) {
        if (si.name == kInterfaceNames[k]) {
          interfaces.emplace_back(info_.sensors[i].name, si.name, &values_[i][k]);
        }
      }
    }
  }
  return interfaces;
}

// ============================================================================
// Lifecycle
// ============================================================================

hardware_interface::CallbackReturn BNO085SensorInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (running_) {
    return hardware_interface::CallbackReturn::SUCCESS;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = Latest{};
  }
  for (auto & v : values_) {
    v.fill(kNaN);
  }
  running_ = true;
  thread_ = std::thread(&BNO085SensorInterface::reader_loop, this);
  // Never fail activation because the IMU is missing: the drive hardware must still come
  // up. The reader keeps retrying and the interfaces read NaN until data arrives.
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO085SensorInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  close_i2c();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BNO085SensorInterface::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  close_i2c();
  release_reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// read() — copies the latest sample; never touches the bus
// ============================================================================

hardware_interface::return_type BNO085SensorInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  Latest snap;
  {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) {
      return hardware_interface::return_type::OK;  // keep last values for one cycle
    }
    snap = latest_;
  }

  const auto now = Clock::now();
  auto fresh = [&](bool seen, Clock::time_point t, double rate_hz) {
      if (!seen) {
        return false;
      }
      const auto limit = std::max(
        std::chrono::milliseconds(stale_timeout_ms_),
        std::chrono::milliseconds(static_cast<int>(3000.0 / rate_hz)));
      return now - t <= limit;
    };

  const bool gyro_ok = fresh(snap.seen_gyro, snap.t_gyro, game_rate_hz_);
  const bool accel_ok = fresh(snap.seen_accel, snap.t_accel, game_rate_hz_);
  for (std::size_t i = 0; i < values_.size(); ++i) {
    auto & v = values_[i];
    const bool abs = absolute_[i];
    const bool q_ok = abs ? fresh(snap.seen_rotation, snap.t_rotation, mag_rate_hz_) :
      fresh(snap.seen_game, snap.t_game, game_rate_hz_);
    const double * q = abs ? snap.rotation : snap.game;
    for (std::size_t k = 0; k < 4; ++k) {
      v[k] = q_ok ? q[k] : kNaN;
    }
    for (std::size_t k = 0; k < 3; ++k) {
      v[4 + k] = gyro_ok ? snap.gyro[k] : kNaN;
      v[7 + k] = accel_ok ? snap.accel[k] : kNaN;
    }
  }
  return hardware_interface::return_type::OK;
}

// ============================================================================
// Reader thread
// ============================================================================

void BNO085SensorInterface::reader_loop()
{
  int consecutive_errors = 0;
  std::vector<uint8_t> payload;
  shtp::Header hdr;

  while (running_) {
    if (!initialised_) {
      if (!init_sensor()) {
        RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 5000,
          "BNO085 not responding on /dev/i2c-%d @0x%02X; retrying. IMU values are NaN.",
          i2c_bus_, i2c_address_);
        close_i2c();
        for (int i = 0; i < 10 && running_; ++i) {
          sleep_ms(100);
        }
        continue;
      }
      initialised_ = true;
      consecutive_errors = 0;
    }

    if (need_reenable_.exchange(false)) {
      RCLCPP_WARN(logger_, "BNO085 reported a reset: re-enabling its reports.");
      if (!enable_features()) {
        initialised_ = false;
        continue;
      }
    }

    const int r = read_packet(hdr, payload);
    if (r > 0) {
      consecutive_errors = 0;
      handle_packet(hdr, payload);
      continue;  // drain without sleeping
    }
    if (r < 0) {
      if (++consecutive_errors >= 20) {
        RCLCPP_ERROR(logger_, "BNO085: 20 consecutive I2C errors, re-initialising the sensor.");
        initialised_ = false;
        close_i2c();
        consecutive_errors = 0;
      }
      sleep_ms(5);
      continue;
    }
    consecutive_errors = 0;
    std::this_thread::sleep_for(std::chrono::microseconds(1500));
  }
}

bool BNO085SensorInterface::init_sensor()
{
  close_i2c();
  if (reset_gpio_ >= 0 && !pulse_reset()) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 10000,
      "BNO085 hardware reset via GPIO %d failed; using a soft reset only.", reset_gpio_);
  }
  if (!open_i2c()) {
    return false;
  }
  seq_.fill(0);
  rx_.reserve(kMaxPacket);

  // Waits up to timeout_ms, feeding packets to `pred`; false on timeout / dead bus.
  auto wait_for = [&](int timeout_ms, auto && pred) -> bool {
      const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
      int errors = 0;
      shtp::Header h;
      std::vector<uint8_t> p;
      while (running_ && Clock::now() < deadline) {
        const int r = read_packet(h, p);
        if (r > 0) {
          errors = 0;
          if (pred(h, p)) {
            return true;
          }
        } else if (r < 0) {
          if (++errors >= 10) {
            return false;
          }
          sleep_ms(5);
        } else {
          sleep_ms(2);
        }
      }
      return false;
    };

  // Drain whatever the reset left (advertisement, reset-complete).
  wait_for(300, [](const shtp::Header &, const std::vector<uint8_t> &) {return false;});

  if (!send_packet(shtp::kChannelExecutable, shtp::build_soft_reset())) {
    return false;
  }
  const bool reset_ok = wait_for(2000, [](const shtp::Header & h, const std::vector<uint8_t> & p) {
        return h.channel == shtp::kChannelExecutable && !p.empty() && p[0] == 0x01;
      });
  if (!reset_ok) {
    return false;  // nothing answered: not wired / wrong address / bus down
  }
  wait_for(150, [](const shtp::Header &, const std::vector<uint8_t> &) {return false;});

  if (!send_packet(shtp::kChannelControl, shtp::build_product_id_request())) {
    return false;
  }
  uint8_t sw_major = 0, sw_minor = 0;
  const bool id_ok = wait_for(1000, [&](const shtp::Header & h, const std::vector<uint8_t> & p) {
        if (h.channel == shtp::kChannelControl && p.size() >= 5 &&
        p[0] == shtp::kReportProductIdResponse)
        {
          sw_major = p[2];
          sw_minor = p[3];
          return true;
        }
        return false;
      });
  if (!id_ok) {
    RCLCPP_WARN(logger_, "BNO085: no product ID response (continuing anyway).");
  } else {
    RCLCPP_INFO(logger_, "BNO085 up: SH-2 firmware %u.%u", sw_major, sw_minor);
  }

  return enable_features();
}

bool BNO085SensorInterface::enable_features()
{
  const auto game_us = static_cast<uint32_t>(1.0e6 / game_rate_hz_);
  const auto mag_us = static_cast<uint32_t>(1.0e6 / mag_rate_hz_);
  const struct { uint8_t id; uint32_t us; } features[] = {
    {shtp::kReportAccelerometer, game_us},
    {shtp::kReportGyroscope, game_us},
    {shtp::kReportGameRotation, game_us},
    {shtp::kReportRotationVector, mag_us},
  };
  for (const auto & f : features) {
    if (!send_packet(shtp::kChannelControl, shtp::build_set_feature(f.id, f.us))) {
      return false;
    }
    sleep_ms(5);
  }
  return true;
}

bool BNO085SensorInterface::open_i2c()
{
  const std::string dev = "/dev/i2c-" + std::to_string(i2c_bus_);
  i2c_fd_ = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
  if (i2c_fd_ < 0) {
    RCLCPP_ERROR_THROTTLE(logger_, steady_clock_, 5000, "Cannot open %s: %s (is the user in "
      "the 'i2c' group?)", dev.c_str(), std::strerror(errno));
    return false;
  }
  if (ioctl(i2c_fd_, I2C_SLAVE, i2c_address_) < 0) {
    RCLCPP_ERROR_THROTTLE(logger_, steady_clock_, 5000, "I2C_SLAVE 0x%02X failed: %s",
      i2c_address_, std::strerror(errno));
    close_i2c();
    return false;
  }
  return true;
}

void BNO085SensorInterface::close_i2c()
{
  if (i2c_fd_ >= 0) {
    ::close(i2c_fd_);
    i2c_fd_ = -1;
  }
}

std::string BNO085SensorInterface::find_gpio_chip(int line)
{
  std::string best;
  uint32_t best_lines = 0;
  for (int i = 0; i < 16; ++i) {
    const std::string path = "/dev/gpiochip" + std::to_string(i);
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
      continue;
    }
    gpiochip_info info{};
    if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &info) == 0) {
      const std::string label = info.label;
      // The Pi's SoC GPIO header controller: pinctrl-rp1 (Pi 5), pinctrl-bcm2711 (Pi 4),
      // pinctrl-bcm2835 / pinctrl-bcm2836... (older).
      if (label.find("pinctrl-rp1") != std::string::npos ||
        label.find("pinctrl-bcm") != std::string::npos)
      {
        ::close(fd);
        return path;
      }
      if (info.lines > best_lines && info.lines > static_cast<uint32_t>(line)) {
        best_lines = info.lines;
        best = path;
      }
    }
    ::close(fd);
  }
  return best;
}

bool BNO085SensorInterface::pulse_reset()
{
  release_reset();
  const std::string chip = reset_chip_ == "auto" ? find_gpio_chip(reset_gpio_) : reset_chip_;
  if (chip.empty()) {
    return false;
  }
  const int chip_fd = ::open(chip.c_str(), O_RDONLY | O_CLOEXEC);
  if (chip_fd < 0) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 10000, "Cannot open %s: %s (user in the "
      "'gpio' group?)", chip.c_str(), std::strerror(errno));
    return false;
  }

  gpio_v2_line_request req{};
  req.offsets[0] = static_cast<uint32_t>(reset_gpio_);
  req.num_lines = 1;
  std::strncpy(req.consumer, "agrobot_bno085", sizeof(req.consumer) - 1);
  req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
  req.config.num_attrs = 1;
  req.config.attrs[0].mask = 1;
  req.config.attrs[0].attr.id = GPIO_V2_LINE_ATTR_ID_OUTPUT_VALUES;
  req.config.attrs[0].attr.values = 0;  // start LOW = reset asserted
  const int rc = ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req);
  ::close(chip_fd);
  if (rc < 0) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 10000, "GPIO %d request on %s failed: %s",
      reset_gpio_, chip.c_str(), std::strerror(errno));
    return false;
  }
  reset_fd_ = req.fd;

  sleep_ms(10);
  gpio_v2_line_values vals{};
  vals.mask = 1;
  vals.bits = 1;  // release reset; the line is held HIGH until cleanup
  if (ioctl(reset_fd_, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals) < 0) {
    RCLCPP_WARN(logger_, "GPIO reset release failed: %s", std::strerror(errno));
    release_reset();
    return false;
  }
  sleep_ms(500);  // BNO085 boot time
  return true;
}

void BNO085SensorInterface::release_reset()
{
  if (reset_fd_ >= 0) {
    ::close(reset_fd_);
    reset_fd_ = -1;
  }
}

bool BNO085SensorInterface::send_packet(uint8_t channel, const std::vector<uint8_t> & payload)
{
  if (i2c_fd_ < 0) {
    return false;
  }
  const auto pkt = shtp::build_packet(channel, seq_[channel]++, payload);
  const ssize_t n = ::write(i2c_fd_, pkt.data(), pkt.size());
  return n == static_cast<ssize_t>(pkt.size());
}

int BNO085SensorInterface::read_packet(shtp::Header & hdr, std::vector<uint8_t> & payload)
{
  if (i2c_fd_ < 0) {
    return -1;
  }
  // The sensor re-sends the pending packet from its start on every read transaction,
  // so: peek the 4-byte header for the length, then read the whole packet again.
  uint8_t h[shtp::kHeaderSize];
  if (::read(i2c_fd_, h, sizeof(h)) != static_cast<ssize_t>(sizeof(h))) {
    return -1;
  }
  if (!shtp::parse_header(h, hdr)) {
    return 0;
  }
  if (hdr.length > kMaxPacket) {
    return -1;
  }
  rx_.resize(hdr.length);
  if (::read(i2c_fd_, rx_.data(), hdr.length) != static_cast<ssize_t>(hdr.length)) {
    return -1;
  }
  payload.assign(rx_.begin() + shtp::kHeaderSize, rx_.end());
  return 1;
}

void BNO085SensorInterface::handle_packet(
  const shtp::Header & hdr, const std::vector<uint8_t> & payload)
{
  if (hdr.channel == shtp::kChannelInput && !hdr.continuation) {
    shtp::Sample s;
    shtp::parse_input_reports(payload.data(), payload.size(), s);
    const auto now = Clock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    if (s.has_accel) {
      std::copy(std::begin(s.accel), std::end(s.accel), latest_.accel);
      latest_.seen_accel = true;
      latest_.t_accel = now;
    }
    if (s.has_gyro) {
      std::copy(std::begin(s.gyro), std::end(s.gyro), latest_.gyro);
      latest_.seen_gyro = true;
      latest_.t_gyro = now;
    }
    if (s.has_game) {
      std::copy(std::begin(s.game), std::end(s.game), latest_.game);
      latest_.seen_game = true;
      latest_.t_game = now;
    }
    if (s.has_rotation) {
      std::copy(std::begin(s.rotation), std::end(s.rotation), latest_.rotation);
      latest_.seen_rotation = true;
      latest_.t_rotation = now;
    }
  } else if (
    hdr.channel == shtp::kChannelExecutable && !payload.empty() && payload[0] == 0x01 &&
    initialised_)
  {
    need_reenable_ = true;  // the sensor rebooted by itself (brownout / watchdog)
  }
}

}  // namespace agrobot_hardware

PLUGINLIB_EXPORT_CLASS(agrobot_hardware::BNO085SensorInterface, hardware_interface::SensorInterface)
