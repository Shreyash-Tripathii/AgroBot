// Copyright 2026 Agrobot

#include "agrobot_hardware/bno085_shtp.hpp"

namespace agrobot_hardware
{
namespace shtp
{

namespace
{

int16_t le16(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0] | (p[1] << 8)));
}

// Total length (bytes) of the report types we may see; 0 = unknown.
std::size_t report_length(uint8_t id)
{
  switch (id) {
    case kReportBaseTimestamp:
    case kReportTimestampRebase: return 5;
    case 0x01:  // accelerometer
    case 0x02:  // gyroscope (calibrated)
    case 0x03:  // magnetic field (calibrated)
    case 0x04:  // linear acceleration
    case 0x06: return 10;  // gravity
    case 0x07: return 16;  // gyroscope uncalibrated
    case 0x05:  // rotation vector
    case 0x09:  // geomagnetic rotation vector
    case 0x28:  // ARVR stabilised rotation vector
    case 0x2A: return 14;  // gyro integrated rotation vector
    case 0x08:  // game rotation vector
    case 0x29: return 12;  // ARVR stabilised game rotation vector
    default: return 0;
  }
}

}  // namespace

bool parse_header(const uint8_t * data, Header & out)
{
  const uint16_t raw = static_cast<uint16_t>(data[0] | (data[1] << 8));
  if (raw == 0xFFFF) {
    return false;  // bus idle / no data
  }
  out.continuation = (raw & 0x8000) != 0;
  out.length = raw & 0x7FFF;
  out.channel = data[2];
  out.sequence = data[3];
  return out.length >= kHeaderSize;
}

std::vector<uint8_t> build_packet(
  uint8_t channel, uint8_t sequence, const std::vector<uint8_t> & payload)
{
  const auto total = static_cast<uint16_t>(kHeaderSize + payload.size());
  std::vector<uint8_t> out;
  out.reserve(total);
  out.push_back(static_cast<uint8_t>(total & 0xFF));
  out.push_back(static_cast<uint8_t>((total >> 8) & 0x7F));
  out.push_back(channel);
  out.push_back(sequence);
  out.insert(out.end(), payload.begin(), payload.end());
  return out;
}

std::vector<uint8_t> build_set_feature(uint8_t report_id, uint32_t interval_us)
{
  std::vector<uint8_t> p(17, 0);
  p[0] = kReportSetFeature;
  p[1] = report_id;
  // p[2] feature flags = 0, p[3..4] change sensitivity = 0
  p[5] = static_cast<uint8_t>(interval_us & 0xFF);
  p[6] = static_cast<uint8_t>((interval_us >> 8) & 0xFF);
  p[7] = static_cast<uint8_t>((interval_us >> 16) & 0xFF);
  p[8] = static_cast<uint8_t>((interval_us >> 24) & 0xFF);
  // p[9..12] batch interval = 0, p[13..16] sensor specific config = 0
  return p;
}

std::vector<uint8_t> build_product_id_request()
{
  return {kReportProductIdRequest, 0x00};
}

std::vector<uint8_t> build_soft_reset()
{
  return {0x01};
}

bool parse_input_reports(const uint8_t * payload, std::size_t length, Sample & out)
{
  std::size_t pos = 0;
  while (pos < length) {
    const uint8_t id = payload[pos];
    const std::size_t need = report_length(id);
    if (need == 0 || pos + need > length) {
      return false;  // unknown id or truncated report
    }
    const uint8_t * r = payload + pos;
    switch (id) {
      case kReportAccelerometer:
        for (int i = 0; i < 3; ++i) {
          out.accel[i] = le16(r + 4 + 2 * i) / 256.0;  // Q8
        }
        out.has_accel = true;
        break;
      case kReportGyroscope:
        for (int i = 0; i < 3; ++i) {
          out.gyro[i] = le16(r + 4 + 2 * i) / 512.0;  // Q9
        }
        out.has_gyro = true;
        break;
      case kReportGameRotation:
        for (int i = 0; i < 4; ++i) {
          out.game[i] = le16(r + 4 + 2 * i) / 16384.0;  // Q14: i, j, k, real
        }
        out.has_game = true;
        break;
      case kReportRotationVector:
        for (int i = 0; i < 4; ++i) {
          out.rotation[i] = le16(r + 4 + 2 * i) / 16384.0;
        }
        out.has_rotation = true;
        break;
      default:
        break;  // timestamps and reports we did not ask for: skip
    }
    pos += need;
  }
  return true;
}

}  // namespace shtp
}  // namespace agrobot_hardware
