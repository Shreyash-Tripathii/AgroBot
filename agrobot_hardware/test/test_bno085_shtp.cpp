// Copyright 2026 Agrobot

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "agrobot_hardware/bno085_shtp.hpp"

using namespace agrobot_hardware::shtp;  // NOLINT

namespace
{
void put16(std::vector<uint8_t> & v, int16_t x)
{
  v.push_back(static_cast<uint8_t>(x & 0xFF));
  v.push_back(static_cast<uint8_t>((x >> 8) & 0xFF));
}
}  // namespace

TEST(Shtp, HeaderRoundTrip)
{
  const auto pkt = build_packet(kChannelControl, 7, {1, 2, 3});
  ASSERT_EQ(pkt.size(), 7u);
  Header h;
  ASSERT_TRUE(parse_header(pkt.data(), h));
  EXPECT_EQ(h.length, 7);
  EXPECT_EQ(h.channel, kChannelControl);
  EXPECT_EQ(h.sequence, 7);
  EXPECT_FALSE(h.continuation);
}

TEST(Shtp, HeaderNoData)
{
  const uint8_t zeros[4] = {0, 0, 0, 0};
  const uint8_t ones[4] = {0xFF, 0xFF, 0xFF, 0xFF};
  Header h;
  EXPECT_FALSE(parse_header(zeros, h));
  EXPECT_FALSE(parse_header(ones, h));
}

TEST(Shtp, HeaderContinuationBit)
{
  const uint8_t d[4] = {0x10, 0x80, 0x03, 0x01};
  Header h;
  ASSERT_TRUE(parse_header(d, h));
  EXPECT_TRUE(h.continuation);
  EXPECT_EQ(h.length, 0x10);
}

TEST(Shtp, SetFeatureLayout)
{
  const auto p = build_set_feature(kReportGameRotation, 10000);  // 100 Hz
  ASSERT_EQ(p.size(), 17u);
  EXPECT_EQ(p[0], 0xFD);
  EXPECT_EQ(p[1], 0x08);
  EXPECT_EQ(p[5], 0x10);  // 10000 = 0x2710 little endian
  EXPECT_EQ(p[6], 0x27);
  EXPECT_EQ(p[7], 0x00);
  EXPECT_EQ(p[8], 0x00);
}

TEST(Shtp, ParsesAllReportsWithTimestamp)
{
  std::vector<uint8_t> pl = {kReportBaseTimestamp, 0, 0, 0, 0};
  // accel: 1.0, -2.0, 9.5 m/s^2 (Q8)
  pl.insert(pl.end(), {kReportAccelerometer, 1, 0, 0});
  put16(pl, 256); put16(pl, -512); put16(pl, 2432);
  // gyro: 0.5, -0.25, 1.0 rad/s (Q9)
  pl.insert(pl.end(), {kReportGyroscope, 2, 0, 0});
  put16(pl, 256); put16(pl, -128); put16(pl, 512);
  // game rotation: i=0.5, j=0, k=0.5, real=0.5 (Q14)
  pl.insert(pl.end(), {kReportGameRotation, 3, 0, 0});
  put16(pl, 8192); put16(pl, 0); put16(pl, 8192); put16(pl, 8192);
  // rotation vector + accuracy
  pl.insert(pl.end(), {kReportRotationVector, 4, 0, 0});
  put16(pl, 0); put16(pl, 0); put16(pl, 16384); put16(pl, 0); put16(pl, 100);

  Sample s;
  ASSERT_TRUE(parse_input_reports(pl.data(), pl.size(), s));
  EXPECT_TRUE(s.has_accel && s.has_gyro && s.has_game && s.has_rotation);
  EXPECT_DOUBLE_EQ(s.accel[0], 1.0);
  EXPECT_DOUBLE_EQ(s.accel[1], -2.0);
  EXPECT_DOUBLE_EQ(s.accel[2], 9.5);
  EXPECT_DOUBLE_EQ(s.gyro[0], 0.5);
  EXPECT_DOUBLE_EQ(s.gyro[1], -0.25);
  EXPECT_DOUBLE_EQ(s.gyro[2], 1.0);
  EXPECT_DOUBLE_EQ(s.game[0], 0.5);
  EXPECT_DOUBLE_EQ(s.game[3], 0.5);
  EXPECT_DOUBLE_EQ(s.rotation[2], 1.0);
}

TEST(Shtp, OnlyPresentReportsAreFlagged)
{
  std::vector<uint8_t> pl = {kReportGyroscope, 0, 0, 0};
  put16(pl, 0); put16(pl, 0); put16(pl, 0);
  Sample s;
  ASSERT_TRUE(parse_input_reports(pl.data(), pl.size(), s));
  EXPECT_TRUE(s.has_gyro);
  EXPECT_FALSE(s.has_accel);
  EXPECT_FALSE(s.has_game);
  EXPECT_FALSE(s.has_rotation);
}

TEST(Shtp, TimestampRebaseIsSkipped)
{
  std::vector<uint8_t> pl = {kReportTimestampRebase, 1, 2, 3, 4};
  pl.insert(pl.end(), {kReportAccelerometer, 0, 0, 0});
  put16(pl, 256); put16(pl, 0); put16(pl, 0);
  Sample s;
  ASSERT_TRUE(parse_input_reports(pl.data(), pl.size(), s));
  EXPECT_DOUBLE_EQ(s.accel[0], 1.0);
}

TEST(Shtp, UnknownIdStopsButKeepsEarlierData)
{
  std::vector<uint8_t> pl = {kReportGyroscope, 0, 0, 0};
  put16(pl, 512); put16(pl, 0); put16(pl, 0);
  pl.insert(pl.end(), {0x7E, 0, 0, 0, 0});
  Sample s;
  EXPECT_FALSE(parse_input_reports(pl.data(), pl.size(), s));
  EXPECT_TRUE(s.has_gyro);
  EXPECT_DOUBLE_EQ(s.gyro[0], 1.0);
}

TEST(Shtp, TruncatedReportIsRejected)
{
  std::vector<uint8_t> pl = {kReportGameRotation, 0, 0, 0};
  put16(pl, 1); put16(pl, 2);  // needs 4 values
  Sample s;
  EXPECT_FALSE(parse_input_reports(pl.data(), pl.size(), s));
  EXPECT_FALSE(s.has_game);
}
