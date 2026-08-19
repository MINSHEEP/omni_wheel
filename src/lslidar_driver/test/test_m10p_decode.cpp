#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "lslidar_driver/m10_packet_decoder.hpp"
#include "lslidar_driver/scan_angle_mask.hpp"

namespace lslidar_driver {
namespace {

constexpr size_t kAngleOffset = 4;
constexpr size_t kDataOffset = 8;
constexpr size_t kSlotCount = 4;

std::array<uint8_t, 32> makePacket(const std::array<uint16_t, kSlotCount>& distances) {
  std::array<uint8_t, 32> packet{};
  constexpr uint16_t kStartAngle = 1000;
  packet[kAngleOffset] = static_cast<uint8_t>(kStartAngle >> 8);
  packet[kAngleOffset + 1] = static_cast<uint8_t>(kStartAngle & 0xFF);

  for (size_t slot_idx = 0; slot_idx < distances.size(); ++slot_idx) {
    const size_t offset = kDataOffset + slot_idx * 2;
    packet[offset] = static_cast<uint8_t>(distances[slot_idx] >> 8);
    packet[offset + 1] = static_cast<uint8_t>(distances[slot_idx] & 0xFF);
  }
  return packet;
}

TEST(M10pDecodeTest, InvalidDistanceKeepsOriginalSlotAngle) {
  const auto packet = makePacket({1000, 0xFFFF, 2000, 3000});

  const auto points = decodeM10PacketSlots(packet.data(), packet.size(), kSlotCount, kAngleOffset,
                                           kDataOffset, 15.0f, 0.001f);

  ASSERT_EQ(points.size(), 3U);
  // 原始第 2 号槽位仍是 7.50° 偏移，不能压缩为过滤后的第 1 号有效点。
  EXPECT_EQ(points[0].azimuth, 1000);
  EXPECT_EQ(points[1].azimuth, 1750);
  EXPECT_EQ(points[2].azimuth, 2125);
}

TEST(M10pDecodeTest, AllInvalidDistancesProduceEmptyPacket) {
  const auto packet = makePacket({0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF});

  const auto points = decodeM10PacketSlots(packet.data(), packet.size(), kSlotCount, kAngleOffset,
                                           kDataOffset, 15.0f, 0.001f);

  EXPECT_TRUE(points.empty());
}

TEST(M10pDecodeTest, ZeroDeclaredSlotsDoNotDivideByZero) {
  const auto packet = makePacket({1000, 2000, 3000, 4000});

  const auto points = decodeM10PacketSlots(packet.data(), packet.size(), 0, kAngleOffset,
                                           kDataOffset, 15.0f, 0.001f);

  EXPECT_TRUE(points.empty());
}

TEST(M10pDecodeTest, TruncatedPacketStopsAtAvailableSlots) {
  const auto packet = makePacket({1000, 2000, 3000, 4000});

  const auto points = decodeM10PacketSlots(packet.data(), kDataOffset + 3, kSlotCount, kAngleOffset,
                                           kDataOffset, 15.0f, 0.001f);

  ASSERT_EQ(points.size(), 1U);
  EXPECT_EQ(points.front().azimuth, 1000);
}

TEST(M10pDecodeTest, ValidatesDeclaredPacketSizeBeforePointCountCalculation) {
  EXPECT_TRUE(isValidM10PPacketSize(158, 1206));
  EXPECT_TRUE(isValidM10PPacketSize(180, 180));
  EXPECT_FALSE(isValidM10PPacketSize(19, 1206));
  EXPECT_FALSE(isValidM10PPacketSize(21, 1206));
  EXPECT_FALSE(isValidM10PPacketSize(182, 1206));
  EXPECT_FALSE(isValidM10PPacketSize(158, 157));
}

TEST(M10pDecodeTest, RealSlotCountKeepsCentidegreePrecisionAcrossWrap) {
  constexpr size_t kRealSlotCount = 75;
  std::array<uint8_t, 180> packet{};
  constexpr uint16_t kStartAngle = 35990;
  packet[kAngleOffset] = static_cast<uint8_t>(kStartAngle >> 8);
  packet[kAngleOffset + 1] = static_cast<uint8_t>(kStartAngle & 0xFF);
  for (size_t slot_idx = 0; slot_idx < kRealSlotCount; ++slot_idx) {
    const size_t offset = kDataOffset + slot_idx * 2;
    packet[offset] = 0x00;
    packet[offset + 1] = 0x01;
  }

  const auto points = decodeM10PacketSlots(packet.data(), packet.size(), kRealSlotCount,
                                           kAngleOffset, kDataOffset, 15.0f, 0.001f);

  ASSERT_EQ(points.size(), kRealSlotCount);
  EXPECT_EQ(points[1].azimuth, 10);
  EXPECT_EQ(points.back().azimuth, 1470);
}

TEST(M10pScanMaskTest, RearBlindSectorIsDisabledWithoutMaskingFront) {
  const std::vector<long int> disable_min{13500};
  const std::vector<long int> disable_max{22500};

  constexpr double kPi = 3.14159265358979323846;
  EXPECT_FALSE(isScanAngleDisabled(0.0, disable_min, disable_max));
  EXPECT_TRUE(isScanAngleDisabled(-kPi, disable_min, disable_max));
  EXPECT_TRUE(isScanAngleDisabled(kPi - 0.01, disable_min, disable_max));
}

TEST(M10pScanMaskTest, DisabledRangeUsesHalfOpenBoundaries) {
  const std::vector<long int> disable_min{13500};
  const std::vector<long int> disable_max{22500};
  constexpr double kPi = 3.14159265358979323846;

  // LaserScan 角度方向与雷达方位角相反：-135° 对应方位角 135°（包含），
  // +135° 对应方位角 225°（结束边界，不包含）。
  EXPECT_TRUE(isScanAngleDisabled(-3.0 * kPi / 4.0, disable_min, disable_max));
  EXPECT_FALSE(isScanAngleDisabled(3.0 * kPi / 4.0, disable_min, disable_max));
}

TEST(M10pScanMaskTest, WrappedRangeAndDisabledSentinelAreHandled) {
  constexpr double kPi = 3.14159265358979323846;
  EXPECT_TRUE(isScanAngleDisabled(0.0, {35000}, {1000}));
  EXPECT_FALSE(isScanAngleDisabled(kPi / 2.0, {35000}, {1000}));
  EXPECT_FALSE(isScanAngleDisabled(0.0, {0}, {0}));
}

TEST(M10pScanMaskTest, MismatchedRangeVectorsDoNotReadPastTheEnd) {
  EXPECT_FALSE(isScanAngleDisabled(0.0, {35000}, {}));
  EXPECT_EQ(scanAngleToLidarCentidegrees(std::nan("")), -1);
}

}  // namespace
}  // namespace lslidar_driver
