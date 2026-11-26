// Copyright 2026 Agrobot
//
// Pure (no I/O) helpers for the BNO08x SHTP / SH-2 protocol: packet framing, the
// handful of commands we send, and decoding of the input sensor reports we use.
// Kept separate from the hardware component so it can be unit tested.

#ifndef AGROBOT_HARDWARE__BNO085_SHTP_HPP_
#define AGROBOT_HARDWARE__BNO085_SHTP_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace agrobot_hardware
{
namespace shtp
{

// SHTP channels
constexpr uint8_t kChannelCommand = 0;     // SHTP advertisement
constexpr uint8_t kChannelExecutable = 1;  // reset / reset-complete
constexpr uint8_t kChannelControl = 2;     // SH-2 commands (set feature, product id)
constexpr uint8_t kChannelInput = 3;       // sensor input reports

// Report ids
constexpr uint8_t kReportAccelerometer = 0x01;   // Q8,  m/s^2
constexpr uint8_t kReportGyroscope = 0x02;       // Q9,  rad/s (calibrated)
constexpr uint8_t kReportRotationVector = 0x05;  // Q14 i,j,k,real + accuracy (Q12), uses mag
constexpr uint8_t kReportGameRotation = 0x08;    // Q14 i,j,k,real, no magnetometer
constexpr uint8_t kReportTimestampRebase = 0xFA;
constexpr uint8_t kReportBaseTimestamp = 0xFB;
constexpr uint8_t kReportSetFeature = 0xFD;
constexpr uint8_t kReportProductIdRequest = 0xF9;
constexpr uint8_t kReportProductIdResponse = 0xF8;

constexpr std::size_t kHeaderSize = 4;

struct Header
{
  uint16_t length{0};  // total packet length INCLUDING the 4 header bytes
  bool continuation{false};
  uint8_t channel{0};
  uint8_t sequence{0};
};

/// Decodes a 4-byte header. Returns false when there is no packet (length 0, 0xFFFF or < 4).
bool parse_header(const uint8_t * data, Header & out);

/// Header + payload, ready to write to the device.
std::vector<uint8_t> build_packet(
  uint8_t channel, uint8_t sequence, const std::vector<uint8_t> & payload);

/// Set Feature Command payload (17 bytes) for `report_id` at `interval_us`.
std::vector<uint8_t> build_set_feature(uint8_t report_id, uint32_t interval_us);

/// Product ID request payload (2 bytes).
std::vector<uint8_t> build_product_id_request();

/// Channel-1 soft reset payload.
std::vector<uint8_t> build_soft_reset();

/// Latest values decoded from one packet. A `has_*` flag is set only if that report
/// was present (so callers can timestamp each field independently).
struct Sample
{
  bool has_accel{false};
  bool has_gyro{false};
  bool has_game{false};
  bool has_rotation{false};
  double accel[3]{0.0, 0.0, 0.0};       // m/s^2
  double gyro[3]{0.0, 0.0, 0.0};        // rad/s
  double game[4]{0.0, 0.0, 0.0, 1.0};   // x, y, z, w
  double rotation[4]{0.0, 0.0, 0.0, 1.0};
};

/// Decodes a channel-3 payload (after the 4 header bytes). Unknown report ids or a
/// truncated report stop the decode and return false; everything decoded before that
/// is kept in `out`.
bool parse_input_reports(const uint8_t * payload, std::size_t length, Sample & out);

}  // namespace shtp
}  // namespace agrobot_hardware

#endif  // AGROBOT_HARDWARE__BNO085_SHTP_HPP_
