/**
 * @file test_protocol.cpp
 * @brief 验证 USB CDC 命令帧、CRC、重同步及轮速/IMU 反馈解码。
 */
#include <gtest/gtest.h>
#include "kiwi_base/robomaster_protocol.hpp"
#include <cstring>
#include <limits>

namespace {
// 写入协议规定的小端序 uint16 字段。
void put_u16(std::vector<uint8_t> & payload, size_t offset, uint16_t value)
{
  payload[offset] = static_cast<uint8_t>(value);
  payload[offset + 1] = static_cast<uint8_t>(value >> 8);
}

// 写入协议规定的小端序 uint32 字段。
void put_u32(std::vector<uint8_t> & payload, size_t offset, uint32_t value)
{
  for (size_t i = 0; i < 4; ++i) payload[offset + i] = static_cast<uint8_t>(value >> (8 * i));
}

// 按主机浮点表示写入一个 float32 字段，测试协议偏移和字节序。
void put_f32(std::vector<uint8_t> & payload, size_t offset, float value)
{
  std::memcpy(payload.data() + offset, &value, sizeof(value));
}
}  // namespace

// 分片和连续拼接的命令帧都应被解析为完整回调。
TEST(Protocol, HandlesPartialAndConcatenatedFrames){kiwi_base::CommandFrame c;c.seq=42;c.timestamp_ms=123;c.enable=true;c.motor_rad_s={1.0f,-2.0f,3.0f};auto bytes=kiwi_base::encode_command(c);int calls=0;kiwi_base::FrameParser p([&](uint8_t v,kiwi_base::FrameType t,const std::vector<uint8_t>&payload){EXPECT_EQ(v,0x33);EXPECT_EQ(t,kiwi_base::FrameType::Command);EXPECT_EQ(payload.size(),12u);++calls;});p.feed(bytes.data(),4);p.feed(bytes.data()+4,bytes.size()-4);p.feed(bytes.data(),bytes.size());EXPECT_EQ(calls,2u);EXPECT_EQ(p.crc_errors(),0u);}
// 修改 payload 后 CRC 不匹配，解析器必须丢弃该帧并计数。
TEST(Protocol, DetectsCrcError){auto b=kiwi_base::encode_command({});b[7]^=0x20;kiwi_base::FrameParser p; p.feed(b.data(),b.size());EXPECT_EQ(p.crc_errors(),1u);}

// 固定零速向量用于锁定计划文档中的协议 golden vector。
TEST(Protocol, MatchesPlanGoldenVector){
  const auto bytes = kiwi_base::encode_command({1, 0, true, {0.0f, 0.0f, 0.0f}});
  const std::vector<uint8_t> expected = {
    0x33, 0x0C, 0x3A, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFD};
  EXPECT_EQ(bytes, expected);
}

// 三路电机速度应以 little-endian float32 顺序编码。
TEST(Protocol, EncodesThreeMotorFloatsLittleEndian){
  const auto bytes = kiwi_base::encode_command({0, 0, true, {1.0f, -2.0f, 3.0f}});
  const std::vector<uint8_t> expected = {
    0x33, 0x0C, 0x3A, 0x65, 0x00, 0x00, 0x80, 0x3F, 0x00,
    0x00, 0x00, 0xC0, 0x00, 0x00, 0x40, 0x40, 0x64, 0x8E};
  EXPECT_EQ(bytes, expected);
}

// 未知数据 ID 不能触发有效帧回调。
TEST(Protocol, RejectsUnknownFrameType){
  auto bytes = kiwi_base::encode_command({});
  bytes[2] = 0x40;
  bytes[3] = kiwi_base::crc8_dji(bytes.data(), 3);
  kiwi_base::FrameParser parser;
  parser.feed(bytes.data(), bytes.size());
  EXPECT_EQ(parser.frames(), 0u);
  EXPECT_GE(parser.dropped_bytes(), 1u);
}

// 命令 ID 使用错误 payload 长度时应被拒绝。
TEST(Protocol, RejectsWrongCommandLength){
  auto bytes = kiwi_base::encode_command({});
  bytes[1] = 0x0B;
  bytes[3] = kiwi_base::crc8_dji(bytes.data(), 3);
  kiwi_base::FrameParser parser;
  parser.feed(bytes.data(), bytes.size());
  EXPECT_EQ(parser.frames(), 0u);
  EXPECT_GE(parser.crc_errors(), 1u);
}

// 软件禁能时，无论调用者传入什么速度，线上都必须发送零速。
TEST(Protocol, DisabledCommandEncodesAllZeros){
  const auto bytes = kiwi_base::encode_command({0, 0, false, {1.0f, -2.0f, 3.0f}});
  const auto zero = kiwi_base::encode_command({0, 0, true, {0.0f, 0.0f, 0.0f}});
  EXPECT_EQ(bytes, zero);
}

// NaN/Inf 等非法命令必须安全降级为零速帧。
TEST(Protocol, NonFiniteCommandFailsSafeToAllZeros){
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto bytes = kiwi_base::encode_command({0, 0, true, {1.0f, nan, 3.0f}});
  const auto zero = kiwi_base::encode_command({0, 0, true, {0.0f, 0.0f, 0.0f}});
  EXPECT_EQ(bytes, zero);
}

// 反馈解析路径同样拒绝包含非有限浮点值的帧。
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

// IMU 时间戳按四字节无符号小端整数解码，不能截断为 uint16。
TEST(Protocol, DecodesImuTimestampAsUint32){
  std::vector<uint8_t> payload(34, 0);
  payload[0] = 7;
  payload[2] = 0x78; payload[3] = 0x56; payload[4] = 0x34; payload[5] = 0x12;
  kiwi_base::ImuStateFrame imu;
  ASSERT_TRUE(kiwi_base::decode_imu_state(payload, imu));
  EXPECT_EQ(imu.seq, 7);
  EXPECT_EQ(imu.timestamp_ms, 0x12345678u);
}

// 验证轮状态各字段偏移、符号和电池/故障字段的解码结果。
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

// 使用实机抓到的完整 0x3B 帧锁定 ID、长度、CRC、分片处理和十二浮点顺序。
TEST(Protocol, ParsesMotorImuFeedbackGoldenFrame){
  const std::vector<uint8_t> frame = {
    0x33, 0x30, 0x3B, 0x5B,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x47, 0xC0,
    0x5C, 0xE2, 0x70, 0x3F, 0x95, 0x3F, 0xB1, 0xBC,
    0xE7, 0xAE, 0xBC, 0x3B, 0x58, 0xA0, 0x8B, 0x3A,
    0x5A, 0xC0, 0x42, 0x3B, 0x12, 0x73, 0xFE, 0xC0,
    0xB9, 0x33, 0x55, 0x3D, 0x46, 0x4F, 0xBA, 0xC0,
    0xEC, 0x74};
  std::vector<uint8_t> payload;
  std::vector<uint8_t> observed_raw;
  kiwi_base::FrameParser parser(
    [&](uint8_t version, kiwi_base::FrameType type, const std::vector<uint8_t> & value) {
      EXPECT_EQ(version, kiwi_base::kProtocolVersion);
      EXPECT_EQ(type, kiwi_base::FrameType::MotorImuFeedback);
      payload = value;
    },
    [&](const std::vector<uint8_t> & value) {observed_raw = value;});
  parser.feed(frame.data(), 19);
  parser.feed(frame.data() + 19, frame.size() - 19);
  ASSERT_EQ(parser.frames(), 1U);
  EXPECT_EQ(observed_raw, frame);

  kiwi_base::MotorImuFeedbackFrame state;
  ASSERT_TRUE(kiwi_base::decode_motor_imu_feedback(payload, state));
  EXPECT_FLOAT_EQ(state.motor_actual[0], 0.0F);
  EXPECT_NEAR(state.euler[0], -3.11328125F, 1e-6F);
  EXPECT_NEAR(state.euler[1], 0.94095397F, 1e-6F);
  EXPECT_NEAR(state.euler[2], -0.021636764F, 1e-6F);
  EXPECT_NEAR(state.gyro[0], 0.005758155F, 1e-6F);
  EXPECT_NEAR(state.accel[0], -7.95154667F, 1e-6F);
  EXPECT_NEAR(state.accel[2], -5.82217693F, 1e-6F);
}

// 0x3B 解码器必须拒绝错误长度和非有限传感器值。
TEST(Protocol, RejectsInvalidMotorImuFeedbackPayload){
  std::vector<uint8_t> payload(kiwi_base::kMotorImuFeedbackPayloadLength, 0U);
  kiwi_base::MotorImuFeedbackFrame state;
  ASSERT_TRUE(kiwi_base::decode_motor_imu_feedback(payload, state));
  payload.pop_back();
  EXPECT_FALSE(kiwi_base::decode_motor_imu_feedback(payload, state));
  payload.resize(kiwi_base::kMotorImuFeedbackPayloadLength, 0U);
  put_f32(payload, 20, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(kiwi_base::decode_motor_imu_feedback(payload, state));
}
