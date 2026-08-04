#include <gtest/gtest.h>
#include "kiwi_base/robomaster_protocol.hpp"
#include <cstring>
#include <limits>

namespace {
void put_u16(std::vector<uint8_t> & payload, size_t offset, uint16_t value)
{
  payload[offset] = static_cast<uint8_t>(value);
  payload[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void put_u32(std::vector<uint8_t> & payload, size_t offset, uint32_t value)
{
  for (size_t i = 0; i < 4; ++i) payload[offset + i] = static_cast<uint8_t>(value >> (8 * i));
}

void put_f32(std::vector<uint8_t> & payload, size_t offset, float value)
{
  std::memcpy(payload.data() + offset, &value, sizeof(value));
}
}  // namespace

TEST(Protocol, HandlesPartialAndConcatenatedFrames){kiwi_base::CommandFrame c;c.seq=42;c.timestamp_ms=123;c.enable=true;c.motor_rad_s={1.0f,-2.0f,3.0f};auto bytes=kiwi_base::encode_command(c);int calls=0;kiwi_base::FrameParser p([&](uint8_t v,kiwi_base::FrameType t,const std::vector<uint8_t>&payload){EXPECT_EQ(v,0x33);EXPECT_EQ(t,kiwi_base::FrameType::Command);EXPECT_EQ(payload.size(),12u);++calls;});p.feed(bytes.data(),4);p.feed(bytes.data()+4,bytes.size()-4);p.feed(bytes.data(),bytes.size());EXPECT_EQ(calls,2u);EXPECT_EQ(p.crc_errors(),0u);}
TEST(Protocol, DetectsCrcError){auto b=kiwi_base::encode_command({});b[7]^=0x20;kiwi_base::FrameParser p; p.feed(b.data(),b.size());EXPECT_EQ(p.crc_errors(),1u);}

TEST(Protocol, MatchesPlanGoldenVector){
  const auto bytes = kiwi_base::encode_command({1, 0, true, {0.0f, 0.0f, 0.0f}});
  const std::vector<uint8_t> expected = {
    0x33, 0x0C, 0x3A, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFD};
  EXPECT_EQ(bytes, expected);
}

TEST(Protocol, EncodesThreeMotorFloatsLittleEndian){
  const auto bytes = kiwi_base::encode_command({0, 0, true, {1.0f, -2.0f, 3.0f}});
  const std::vector<uint8_t> expected = {
    0x33, 0x0C, 0x3A, 0x65, 0x00, 0x00, 0x80, 0x3F, 0x00,
    0x00, 0x00, 0xC0, 0x00, 0x00, 0x40, 0x40, 0x64, 0x8E};
  EXPECT_EQ(bytes, expected);
}

TEST(Protocol, RejectsUnknownFrameType){
  auto bytes = kiwi_base::encode_command({});
  bytes[2] = 0x40;
  bytes[3] = kiwi_base::crc8_dji(bytes.data(), 3);
  kiwi_base::FrameParser parser;
  parser.feed(bytes.data(), bytes.size());
  EXPECT_EQ(parser.frames(), 0u);
  EXPECT_GE(parser.dropped_bytes(), 1u);
}

TEST(Protocol, RejectsWrongCommandLength){
  auto bytes = kiwi_base::encode_command({});
  bytes[1] = 0x0B;
  bytes[3] = kiwi_base::crc8_dji(bytes.data(), 3);
  kiwi_base::FrameParser parser;
  parser.feed(bytes.data(), bytes.size());
  EXPECT_EQ(parser.frames(), 0u);
  EXPECT_GE(parser.crc_errors(), 1u);
}

TEST(Protocol, DisabledCommandEncodesAllZeros){
  const auto bytes = kiwi_base::encode_command({0, 0, false, {1.0f, -2.0f, 3.0f}});
  const auto zero = kiwi_base::encode_command({0, 0, true, {0.0f, 0.0f, 0.0f}});
  EXPECT_EQ(bytes, zero);
}

TEST(Protocol, NonFiniteCommandFailsSafeToAllZeros){
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto bytes = kiwi_base::encode_command({0, 0, true, {1.0f, nan, 3.0f}});
  const auto zero = kiwi_base::encode_command({0, 0, true, {0.0f, 0.0f, 0.0f}});
  EXPECT_EQ(bytes, zero);
}

TEST(Protocol, RejectsNonFiniteMotorSpeed){
  auto bytes = kiwi_base::encode_command({0, 0, true, {1.0f, 2.0f, 3.0f}});
  const float nan = std::numeric_limits<float>::quiet_NaN();
  std::memcpy(bytes.data() + 4, &nan, sizeof(nan));
  const auto crc = kiwi_base::crc16_ccitt(bytes.data(), bytes.size() - 2);
  bytes[bytes.size() - 2] = static_cast<uint8_t>(crc);
  bytes[bytes.size() - 1] = static_cast<uint8_t>(crc >> 8);
  kiwi_base::FrameParser parser;
  parser.feed(bytes.data(), bytes.size());
  EXPECT_EQ(parser.frames(), 0u);
  EXPECT_GE(parser.crc_errors(), 1u);
}

TEST(Protocol, DecodesImuTimestampAsUint32){
  std::vector<uint8_t> payload(34, 0);
  payload[0] = 7;
  payload[2] = 0x78; payload[3] = 0x56; payload[4] = 0x34; payload[5] = 0x12;
  kiwi_base::ImuStateFrame imu;
  ASSERT_TRUE(kiwi_base::decode_imu_state(payload, imu));
  EXPECT_EQ(imu.seq, 7);
  EXPECT_EQ(imu.timestamp_ms, 0x12345678u);
}

TEST(Protocol, DecodesWheelStateFieldOffsets){
  std::vector<uint8_t> payload(46, 0);
  put_u16(payload, 0, 0x1234);
  put_u16(payload, 2, 0x5678);
  put_u32(payload, 4, 0x89abcdefU);
  put_f32(payload, 8, 1.25F);
  put_f32(payload, 20, -2.5F);
  put_u16(payload, 32, static_cast<uint16_t>(-321));
  put_u16(payload, 38, 24500U);
  put_u32(payload, 40, 0xaabbccddU);
  put_u16(payload, 44, 0x0009U);
  kiwi_base::WheelStateFrame state;
  ASSERT_TRUE(kiwi_base::decode_wheel_state(payload, state));
  EXPECT_EQ(state.state_seq, 0x1234U);
  EXPECT_EQ(state.last_cmd_seq, 0x5678U);
  EXPECT_EQ(state.timestamp_ms, 0x89abcdefU);
  EXPECT_FLOAT_EQ(state.position[0], 1.25F);
  EXPECT_FLOAT_EQ(state.velocity[0], -2.5F);
  EXPECT_EQ(state.current_raw[0], -321);
  EXPECT_EQ(state.battery_mv, 24500U);
  EXPECT_EQ(state.fault_bits, 0xaabbccddU);
  EXPECT_EQ(state.flags, 0x0009U);
}
