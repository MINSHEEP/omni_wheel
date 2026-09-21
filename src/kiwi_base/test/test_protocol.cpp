/**
 * @file test_protocol.cpp
 * @brief 验证 0x3A 命令帧和历史兼容 0x3B/48 字节帧。
 */
#include "kiwi_base/robomaster_protocol.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace
{
// 按协议 little-endian 写入一个 float32，便于直接构造 payload。
void put_f32(std::vector<uint8_t> &payload, size_t offset, float value)
{
  std::memcpy(payload.data() + offset, &value, sizeof(value));
}

// 构造带 CRC8/CRC16 的历史兼容帧，只用于离线协议回归。
std::vector<uint8_t> make_feedback(const std::array<float, 12> &values)
{
  std::vector<uint8_t> frame{
    kiwi_base::kProtocolVersion,
    kiwi_base::kFeedbackPayloadLength,
    static_cast<uint8_t>(kiwi_base::FrameType::Feedback),
    0};
  frame[3] = kiwi_base::crc8_dji(frame.data(), 3);
  frame.resize(4 + kiwi_base::kFeedbackPayloadLength);
  for (size_t i = 0; i < values.size(); ++i)
    put_f32(frame, 4 + i * sizeof(float), values[i]);
  const uint16_t crc = kiwi_base::crc16_ccitt(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc));
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  return frame;
}
}  // namespace

namespace
{
// 独立固件布局向量：timestamp=0x12345678，通道 -660/0/660/-1，拨杆中/上。
const std::vector<uint8_t> kRemotePayload{
  0x78, 0x56, 0x34, 0x12, 0x6C, 0xFD, 0x00, 0x00,
  0x94, 0x02, 0xFF, 0xFF, 0x03, 0x01};

std::vector<uint8_t> make_remote(std::vector<uint8_t> payload = kRemotePayload)
{
  std::vector<uint8_t> frame{0x33, static_cast<uint8_t>(payload.size()), 0x3C};
  frame.push_back(kiwi_base::crc8_dji(frame.data(), frame.size()));
  frame.insert(frame.end(), payload.begin(), payload.end());
  const auto crc = kiwi_base::crc16_ccitt(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc));
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  return frame;
}
}  // namespace

TEST(Protocol, DecodesRemoteSignedChannelsAndChecksRanges)
{
  kiwi_base::RemoteControlFrame remote;
  ASSERT_TRUE(kiwi_base::decode_remote_control(kRemotePayload, remote));
  EXPECT_EQ(remote.timestamp_ms, 0x12345678U);
  EXPECT_EQ(remote.channels, (std::array<int16_t, 4>{-660, 0, 660, -1}));
  EXPECT_EQ(remote.sw_right, 3);
  EXPECT_EQ(remote.sw_left, 1);
  EXPECT_TRUE(kiwi_base::remote_controls_valid(remote));
  remote.channels[0] = -701;
  EXPECT_FALSE(kiwi_base::remote_controls_valid(remote));
  remote.channels[0] = 701;
  EXPECT_FALSE(kiwi_base::remote_controls_valid(remote));
  remote.channels[0] = -700;
  remote.sw_left = 0;
  EXPECT_FALSE(kiwi_base::remote_controls_valid(remote));
  remote.sw_left = 4;
  EXPECT_FALSE(kiwi_base::remote_controls_valid(remote));
  EXPECT_FALSE(kiwi_base::decode_remote_control({}, remote));
}

TEST(Protocol, ParsesMixedFramesAtEverySplitWithoutCrossDispatch)
{
  const auto feedback = make_feedback({});
  const auto remote = make_remote();
  std::vector<uint8_t> stream = feedback;
  stream.insert(stream.end(), remote.begin(), remote.end());
  stream.insert(stream.end(), feedback.begin(), feedback.end());
  for (size_t split = 0; split <= stream.size(); ++split)
  {
    size_t feedback_count = 0, raw_count = 0, remote_count = 0;
    kiwi_base::FrameParser parser(
      [&](uint8_t, const std::vector<uint8_t> &payload)
      { ++feedback_count; EXPECT_EQ(payload.size(), 48U); },
      [&](const std::vector<uint8_t> &frame)
      { ++raw_count; EXPECT_EQ(frame, feedback); },
      [&](const kiwi_base::RemoteControlFrame &frame)
      { ++remote_count; EXPECT_EQ(frame.channels[0], -660); });
    parser.feed(stream.data(), split);
    parser.feed(stream.data() + split, stream.size() - split);
    EXPECT_EQ(feedback_count, 2U);
    EXPECT_EQ(raw_count, 2U);
    EXPECT_EQ(remote_count, 1U);
    EXPECT_EQ(parser.frames(), 2U);
    EXPECT_EQ(parser.remote_frames(), 1U);
    EXPECT_EQ(parser.errors(), 0U);
  }
}

TEST(Protocol, RejectsBadRemoteCrcAndLengthThenRecovers)
{
  const auto good = make_remote();
  auto bad_header = good;
  bad_header[3] ^= 1;
  auto bad_payload = good;
  bad_payload[5] ^= 1;
  auto wrong_payload = kRemotePayload;
  wrong_payload.push_back(0);
  const auto wrong_length = make_remote(wrong_payload);
  size_t count = 0;
  kiwi_base::FrameParser parser({}, {},
    [&](const kiwi_base::RemoteControlFrame &) { ++count; });
  for (const auto &frame : {bad_header, bad_payload, wrong_length, good})
    for (const auto byte : frame)
      parser.feed(&byte, 1);
  EXPECT_EQ(count, 1U);
  EXPECT_EQ(parser.remote_frames(), 1U);
  EXPECT_GE(parser.errors(), 3U);
  parser.feed(good.data(), 7);
  parser.reset();
  parser.feed(good.data(), good.size());
  EXPECT_EQ(count, 2U);
}

TEST(Protocol, PreservesInvalidControlValuesForDiagnostics)
{
  auto payload = kRemotePayload;
  payload[4] = 0;
  payload[5] = 0x80;  // int16 最小值，不能按浮点判定或静默归零。
  payload[12] = 0;
  const auto frame = make_remote(payload);
  bool seen = false;
  kiwi_base::FrameParser parser({}, {},
    [&](const kiwi_base::RemoteControlFrame &remote)
    {
      seen = true;
      EXPECT_EQ(remote.channels[0], -32768);
      EXPECT_FALSE(kiwi_base::remote_controls_valid(remote));
    });
  parser.feed(frame.data(), frame.size());
  EXPECT_TRUE(seen);
  EXPECT_EQ(parser.errors(), 0U);
}

TEST(Protocol, EncodesZeroCommandGoldenVector)
{
  const auto bytes = kiwi_base::encode_command({true, {0.0F, 0.0F, 0.0F}});
  const std::vector<uint8_t> expected = {
    0x33, 0x0C, 0x3A, 0x65, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFD};
  EXPECT_EQ(bytes, expected);
}

TEST(Protocol, EncodesThreeMotorFloatsLittleEndian)
{
  const auto bytes = kiwi_base::encode_command({true, {1.0F, -2.0F, 3.0F}});
  const std::vector<uint8_t> expected = {
    0x33, 0x0C, 0x3A, 0x65, 0x00, 0x00, 0x80, 0x3F, 0x00,
    0x00, 0x00, 0xC0, 0x00, 0x00, 0x40, 0x40, 0x64, 0x8E};
  EXPECT_EQ(bytes, expected);
}

TEST(Protocol, DisabledAndNonFiniteCommandsBecomeZero)
{
  const auto zero = kiwi_base::encode_command({true, {0.0F, 0.0F, 0.0F}});
  EXPECT_EQ(kiwi_base::encode_command({false, {1.0F, -2.0F, 3.0F}}), zero);
  const float nan = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(kiwi_base::encode_command({true, {1.0F, nan, 3.0F}}), zero);
}

TEST(Protocol, ParsesSplitAndConcatenatedFeedbackFrames)
{
  std::array<float, 12> values{};
  for (size_t i = 0; i < values.size(); ++i)
    values[i] = static_cast<float>(i + 1);
  const auto frame = make_feedback(values);
  std::vector<uint8_t> payload;
  std::vector<uint8_t> raw;
  kiwi_base::FrameParser parser(
    [&](uint8_t version, const std::vector<uint8_t> &value)
    {
      EXPECT_EQ(version, kiwi_base::kProtocolVersion);
      payload = value;
    },
    [&](const std::vector<uint8_t> &value) { raw = value; });

  parser.feed(frame.data(), 9);
  EXPECT_EQ(parser.frames(), 0U);
  parser.feed(frame.data() + 9, frame.size() - 9);
  parser.feed(frame.data(), frame.size());
  EXPECT_EQ(parser.frames(), 2U);
  EXPECT_EQ(payload.size(), kiwi_base::kFeedbackPayloadLength);
  EXPECT_EQ(raw, frame);
}

TEST(Protocol, RejectsCommandAndWrongFeedbackLength)
{
  const auto command = kiwi_base::encode_command({true, {1.0F, 2.0F, 3.0F}});
  kiwi_base::FrameParser parser;
  parser.feed(command.data(), command.size());
  EXPECT_EQ(parser.frames(), 0U);
  EXPECT_GE(parser.errors(), 1U);

  auto wrong = command;
  wrong[1] = kiwi_base::kFeedbackPayloadLength;
  wrong[2] = static_cast<uint8_t>(kiwi_base::FrameType::Feedback);
  wrong[3] = kiwi_base::crc8_dji(wrong.data(), 3);
  parser.feed(wrong.data(), wrong.size());
  EXPECT_EQ(parser.frames(), 0U);
}

TEST(Protocol, DetectsFeedbackCrcError)
{
  const auto good = make_feedback({});
  auto broken = good;
  broken[10] ^= 0x01;
  kiwi_base::FrameParser parser;
  parser.feed(broken.data(), broken.size());
  EXPECT_EQ(parser.frames(), 0U);
  EXPECT_GE(parser.errors(), 1U);
}

TEST(Protocol, DecodesFeedbackFieldOrder)
{
  const std::array<float, 12> values = {
    1.0F, 2.0F, 3.0F,
    0.1F, 0.2F, 0.3F,
    4.0F, 5.0F, 6.0F,
    7.0F, 8.0F, 9.0F};
  std::vector<uint8_t> payload(kiwi_base::kFeedbackPayloadLength, 0);
  for (size_t i = 0; i < values.size(); ++i)
    put_f32(payload, i * sizeof(float), values[i]);

  kiwi_base::FeedbackFrame state;
  ASSERT_TRUE(kiwi_base::decode_feedback(payload, state));
  EXPECT_EQ(state.motor_actual, (std::array<float, 3>{1.0F, 2.0F, 3.0F}));
  EXPECT_EQ(state.euler, (std::array<float, 3>{0.1F, 0.2F, 0.3F}));
  EXPECT_EQ(state.gyro, (std::array<float, 3>{4.0F, 5.0F, 6.0F}));
  EXPECT_EQ(state.accel, (std::array<float, 3>{7.0F, 8.0F, 9.0F}));
}

TEST(Protocol, RejectsInvalidFeedbackPayload)
{
  kiwi_base::FeedbackFrame state;
  std::vector<uint8_t> payload(kiwi_base::kFeedbackPayloadLength, 0);
  ASSERT_TRUE(kiwi_base::decode_feedback(payload, state));
  payload.pop_back();
  EXPECT_FALSE(kiwi_base::decode_feedback(payload, state));
  payload.resize(kiwi_base::kFeedbackPayloadLength, 0);
  put_f32(payload, 20, std::numeric_limits<float>::quiet_NaN());
  EXPECT_FALSE(kiwi_base::decode_feedback(payload, state));
}
