// bno085_shtp_node.cpp
// High-performance C++ ROS 2 driver for BNO085 over UART-SHTP (3,000,000 baud).
// Includes native Linux character device GPIO hardware pulse on RST (GPIO 22).

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <asm/termbits.h>
#include <linux/gpio.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

namespace
{
constexpr uint8_t FLAG_BYTE = 0x7E;
constexpr uint8_t ESCAPE_BYTE = 0x7D;
constexpr uint8_t PROTOCOL_SHTP = 0x01;

constexpr uint8_t CHANNEL_EXECUTABLE = 1;
constexpr uint8_t CHANNEL_CONTROL = 2;
constexpr uint8_t CHANNEL_REPORTS = 3;

constexpr uint8_t SHTP_REPORT_SET_FEATURE_CMD = 0xFD;
constexpr uint8_t SHTP_REPORT_GET_FEATURE_RESP = 0xFC;
constexpr uint8_t SHTP_REPORT_BASE_TIMESTAMP = 0xFB;

constexpr uint8_t REPORT_GYROSCOPE = 0x02;
constexpr uint8_t REPORT_ROTATION_VECTOR = 0x05;       // /imu/data_mag
constexpr uint8_t REPORT_GAME_ROTATION_VECTOR = 0x08;  // /imu/data_game

constexpr double SCALE_QUAT = 1.0 / (1 << 14);  // 2^-14
constexpr double SCALE_GYRO = 1.0 / (1 << 9);   // 2^-9 rad/s
}  // namespace

class BNO085SHTPNode : public rclcpp::Node
{
public:
  BNO085SHTPNode()
  : Node("bno085_shtp_node"), fd_(-1), running_(true)
  {
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyAMA0");
    baudrate_ = this->declare_parameter<int>("baudrate", 3000000);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    game_rate_hz_ = this->declare_parameter<double>("game_rate_hz", 100.0);
    mag_rate_hz_ = this->declare_parameter<double>("mag_rate_hz", 50.0);
    reset_gpio_pin_ = this->declare_parameter<int>("reset_gpio_pin", 4);

    game_interval_us_ = static_cast<uint32_t>(1000000.0 / game_rate_hz_);
    mag_interval_us_ = static_cast<uint32_t>(1000000.0 / mag_rate_hz_);

    pub_game_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_game", 10);
    pub_mag_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_mag", 10);

    worker_ = std::thread(&BNO085SHTPNode::serialWorker, this);
  }

  ~BNO085SHTPNode() override
  {
    running_ = false;
    if (worker_.joinable()) {
      worker_.join();
    }
    closeSerial();
  }

private:
  // Native Linux GPIO v2 character-device line pulse (Active-Low Reset)
  bool pulseHardwareResetPin(int gpio_offset)
  {
    // RPi 4/5 base GPIO chip is /dev/gpiochip0 or /dev/gpiochip4
    const char * chip_paths[] = {"/dev/gpiochip0", "/dev/gpiochip4"};
    int chip_fd = -1;

    for (const char * path : chip_paths) {
      chip_fd = ::open(path, O_RDWR);
      if (chip_fd >= 0) break;
    }

    if (chip_fd < 0) {
      RCLCPP_WARN(this->get_logger(), "Cannot open /dev/gpiochip* for hardware reset: %s", strerror(errno));
      return false;
    }

    struct gpio_v2_line_request req {};
    req.offsets[0] = static_cast<uint32_t>(gpio_offset);
    req.num_lines = 1;
    req.config.flags = GPIO_V2_LINE_FLAG_OUTPUT;
    std::strncpy(req.consumer, "bno085_reset", sizeof(req.consumer) - 1);

    if (::ioctl(chip_fd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
      ::close(chip_fd);
      return false;
    }

    struct gpio_v2_line_values vals {};
    vals.mask = 1ULL;

    // 1. Pull RST LOW (0V) to hold sensor in hardware reset
    vals.bits = 0ULL;
    ::ioctl(req.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // 2. Release RST HIGH (3.3V) to reboot internal Cortex-M0+ processor
    vals.bits = 1ULL;
    ::ioctl(req.fd, GPIO_V2_LINE_SET_VALUES_IOCTL, &vals);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    ::close(req.fd);
    ::close(chip_fd);
    return true;
  }

  bool openSerial()
  {
    closeSerial();

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open %s: %s", port_.c_str(), strerror(errno));
      return false;
    }

    struct termios2 tio {};
    if (::ioctl(fd_, TCGETS2, &tio) < 0) {
      RCLCPP_ERROR(this->get_logger(), "TCGETS2 failed: %s", strerror(errno));
      closeSerial();
      return false;
    }

    tio.c_cflag &= ~CBAUD;
    tio.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tio.c_iflag = IGNPAR;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_ispeed = baudrate_;
    tio.c_ospeed = baudrate_;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (::ioctl(fd_, TCSETS2, &tio) < 0) {
      RCLCPP_ERROR(this->get_logger(), "TCSETS2 failed: %s", strerror(errno));
      closeSerial();
      return false;
    }

    ::ioctl(fd_, TCFLSH, TCIOFLUSH);
    return true;
  }

  void closeSerial()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  void sendFramedShtpPacket(uint8_t channel, const std::vector<uint8_t> & payload)
  {
    if (fd_ < 0) return;

    uint16_t pkt_len = static_cast<uint16_t>(payload.size() + 4);
    uint8_t raw_hdr[4] = {
      static_cast<uint8_t>(pkt_len & 0xFF),
      static_cast<uint8_t>((pkt_len >> 8) & 0x7F),
      channel,
      seq_num_[channel]++
    };

    std::vector<uint8_t> tx;
    tx.reserve(pkt_len * 2 + 8);

    tx.push_back(FLAG_BYTE);
    tx.push_back(FLAG_BYTE);
    tx.push_back(PROTOCOL_SHTP);

    auto append_stuffed = [&tx](uint8_t b) {
      if (b == FLAG_BYTE || b == ESCAPE_BYTE) {
        tx.push_back(ESCAPE_BYTE);
        tx.push_back(b ^ 0x20);
      } else {
        tx.push_back(b);
      }
    };

    for (uint8_t b : raw_hdr) append_stuffed(b);
    for (uint8_t b : payload) append_stuffed(b);

    tx.push_back(FLAG_BYTE);
    ::write(fd_, tx.data(), tx.size());
  }

  void sendSetFeature(uint8_t report_id, uint32_t interval_us)
  {
    std::vector<uint8_t> cmd(17, 0);
    cmd[0] = SHTP_REPORT_SET_FEATURE_CMD;
    cmd[1] = report_id;
    cmd[5] = static_cast<uint8_t>(interval_us & 0xFF);
    cmd[6] = static_cast<uint8_t>((interval_us >> 8) & 0xFF);
    cmd[7] = static_cast<uint8_t>((interval_us >> 16) & 0xFF);
    cmd[8] = static_cast<uint8_t>((interval_us >> 24) & 0xFF);
    sendFramedShtpPacket(CHANNEL_CONTROL, cmd);
  }

  void configureSensor()
  {
    std::memset(seq_num_, 0, sizeof(seq_num_));

    // Hardware pin toggle on GPIO 4 before initiating communications
    RCLCPP_INFO(this->get_logger(), "Power cycling BNO085 hardware reset pin (GPIO %d)...", reset_gpio_pin_);
    pulseHardwareResetPin(reset_gpio_pin_);

    // Drain serial buffer of leftover noise during pin transition
    if (fd_ >= 0) {
      ::ioctl(fd_, TCFLSH, TCIOFLUSH);
      uint8_t dump[2048];
      while (::read(fd_, dump, sizeof(dump)) > 0) {}
    }

    // 1. Send wake-up flags + SHTP Soft Reset
    sendFramedShtpPacket(CHANNEL_EXECUTABLE, {0x01});
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // Drain startup advertisement
    if (fd_ >= 0) {
      uint8_t dump[2048];
      while (::read(fd_, dump, sizeof(dump)) > 0) {}
    }

    // 2. Request Product ID to verify ready state
    sendFramedShtpPacket(CHANNEL_CONTROL, {0xF9, 0x00});
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // 3. Command features: Game RV @ 100 Hz, Mag RV @ 50 Hz, Gyro @ 100 Hz
    for (int i = 0; i < 2; ++i) {
      sendSetFeature(REPORT_GAME_ROTATION_VECTOR, game_interval_us_);
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
      sendSetFeature(REPORT_ROTATION_VECTOR, mag_interval_us_);
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
      sendSetFeature(REPORT_GYROSCOPE, game_interval_us_);
      std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }

    RCLCPP_INFO(
      this->get_logger(),
      "BNO085 configured: Game @ %.0f Hz (%u us), Mag @ %.0f Hz (%u us)",
      game_rate_hz_, game_interval_us_, mag_rate_hz_, mag_interval_us_);
  }

  static int16_t readInt16LE(const uint8_t * ptr)
  {
    return static_cast<int16_t>(
      static_cast<uint16_t>(ptr[0]) | (static_cast<uint16_t>(ptr[1]) << 8));
  }

  static uint32_t readUint32LE(const uint8_t * ptr)
  {
    return static_cast<uint32_t>(ptr[0]) |
           (static_cast<uint32_t>(ptr[1]) << 8) |
           (static_cast<uint32_t>(ptr[2]) << 16) |
           (static_cast<uint32_t>(ptr[3]) << 24);
  }

  void publishImu(
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr & pub,
    double qx, double qy, double qz, double qw,
    const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.orientation_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

    msg.angular_velocity.x = gyro_x_;
    msg.angular_velocity.y = gyro_y_;
    msg.angular_velocity.z = gyro_z_;
    msg.angular_velocity_covariance = {0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01};

    msg.linear_acceleration_covariance[0] = -1.0;
    pub->publish(msg);
  }

  void handleUnstuffedFrame(const std::vector<uint8_t> & frame)
  {
    if (frame.empty() || frame[0] != PROTOCOL_SHTP || frame.size() < 5) return;

    const uint8_t * shtp = frame.data() + 1;
    const size_t shtp_len = frame.size() - 1;
    uint8_t channel = shtp[2];

    const uint8_t * payload = shtp + 4;
    size_t payload_len = shtp_len - 4;

    if (channel == CHANNEL_CONTROL && payload_len >= 9) {
      if (payload[0] == SHTP_REPORT_GET_FEATURE_RESP) {
        uint8_t feat_id = payload[1];
        uint32_t interval_us = readUint32LE(payload + 5);
        double hz = interval_us > 0 ? (1000000.0 / interval_us) : 0.0;
        RCLCPP_INFO(
          this->get_logger(),
          "BNO085 Confirmed Feature 0x%02X: %u us (%.1f Hz)",
          feat_id, interval_us, hz);
      }
      return;
    }

    if (channel != CHANNEL_REPORTS || payload_len < 5) return;

    size_t idx = (payload[0] == SHTP_REPORT_BASE_TIMESTAMP) ? 5 : 0;
    rclcpp::Time stamp = this->get_clock()->now();

    while (idx + 4 <= payload_len) {
      uint8_t report_id = payload[idx];

      if (report_id == REPORT_GAME_ROTATION_VECTOR) {
        if (idx + 12 > payload_len) break;
        double qi = readInt16LE(payload + idx + 4) * SCALE_QUAT;
        double qj = readInt16LE(payload + idx + 6) * SCALE_QUAT;
        double qk = readInt16LE(payload + idx + 8) * SCALE_QUAT;
        double qr = readInt16LE(payload + idx + 10) * SCALE_QUAT;
        publishImu(pub_game_, qi, qj, qk, qr, stamp);
        idx += 12;
      } else if (report_id == REPORT_ROTATION_VECTOR) {
        if (idx + 14 > payload_len) break;
        double qi = readInt16LE(payload + idx + 4) * SCALE_QUAT;
        double qj = readInt16LE(payload + idx + 6) * SCALE_QUAT;
        double qk = readInt16LE(payload + idx + 8) * SCALE_QUAT;
        double qr = readInt16LE(payload + idx + 10) * SCALE_QUAT;
        publishImu(pub_mag_, qi, qj, qk, qr, stamp);
        idx += 14;
      } else if (report_id == REPORT_GYROSCOPE) {
        if (idx + 10 > payload_len) break;
        gyro_x_ = readInt16LE(payload + idx + 4) * SCALE_GYRO;
        gyro_y_ = readInt16LE(payload + idx + 6) * SCALE_GYRO;
        gyro_z_ = readInt16LE(payload + idx + 8) * SCALE_GYRO;
        idx += 10;
      } else if (report_id == SHTP_REPORT_BASE_TIMESTAMP) {
        idx += 5;
      } else {
        break;
      }
    }
  }

  void serialWorker()
  {
    std::vector<uint8_t> current_frame;
    current_frame.reserve(512);
    bool in_escape = false;

    uint8_t rx_chunk[2048];
    auto last_report_time = std::chrono::steady_clock::now();

    while (running_ && rclcpp::ok()) {
      if (fd_ < 0) {
        if (openSerial()) {
          configureSensor();
          last_report_time = std::chrono::steady_clock::now();
        } else {
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
      }

      struct pollfd pfd {};
      pfd.fd = fd_;
      pfd.events = POLLIN;

      int ret = ::poll(&pfd, 1, 40);
      if (ret > 0 && (pfd.revents & POLLIN)) {
        ssize_t n = ::read(fd_, rx_chunk, sizeof(rx_chunk));
        if (n > 0) {
          last_report_time = std::chrono::steady_clock::now();
          for (ssize_t i = 0; i < n; ++i) {
            uint8_t b = rx_chunk[i];
            if (b == FLAG_BYTE) {
              if (!current_frame.empty()) {
                handleUnstuffedFrame(current_frame);
                current_frame.clear();
              }
              in_escape = false;
            } else if (b == ESCAPE_BYTE) {
              in_escape = true;
            } else {
              if (in_escape) {
                b ^= 0x20;
                in_escape = false;
              }
              if (current_frame.size() < 1024) {
                current_frame.push_back(b);
              } else {
                current_frame.clear();
              }
            }
          }
        }
      }

      // Proactive Hardware Power-Cycle Watchdog:
      // If no valid data is received for 1000ms, pull RST to GND and reinitialize.
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_report_time).count() > 1000) {
        RCLCPP_WARN(this->get_logger(), "No BNO085 response for 1s. Performing physical RST power-cycle...");
        configureSensor();
        last_report_time = now;
      }
    }
  }

  std::string port_;
  int baudrate_;
  std::string frame_id_;
  double game_rate_hz_;
  double mag_rate_hz_;
  int reset_gpio_pin_;
  uint32_t game_interval_us_;
  uint32_t mag_interval_us_;

  int fd_;
  std::atomic<bool> running_;
  std::thread worker_;

  uint8_t seq_num_[6] = {0};
  double gyro_x_ = 0.0;
  double gyro_y_ = 0.0;
  double gyro_z_ = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_game_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_mag_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BNO085SHTPNode>());
  rclcpp::shutdown();
  return 0;
}
