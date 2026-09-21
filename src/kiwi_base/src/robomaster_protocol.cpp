/**
 * @file robomaster_protocol.cpp
 * @brief 实现 0x3A 命令组帧、0x3B/0x3C 混合帧解析和 CRC。
 */
#include "kiwi_base/robomaster_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

namespace kiwi_base
{
namespace
{
/** 按 little-endian 写入一个 uint32。 */
void put_u32(std::vector<uint8_t> &bytes, uint32_t value)
{
  for (int i = 0; i < 4; ++i)
    bytes.push_back(static_cast<uint8_t>(value >> (8 * i)));
}

/** 按 IEEE-754 little-endian 写入一个 float32。 */
void put_f32(std::vector<uint8_t> &bytes, float value)
{
  uint32_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  put_u32(bytes, bits);
}

/** 从 little-endian 缓冲区读取 uint16。 */
uint16_t get_u16(const uint8_t *data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(data[1]) << 8;
}

/** 从 little-endian 缓冲区读取 uint32。 */
uint32_t get_u32(const uint8_t *data)
{
  return static_cast<uint32_t>(data[0]) |
         static_cast<uint32_t>(data[1]) << 8 |
         static_cast<uint32_t>(data[2]) << 16 |
         static_cast<uint32_t>(data[3]) << 24;
}

/** 从 little-endian 缓冲区读取 float32。 */
float get_f32(const uint8_t *data)
{
  const uint32_t bits = get_u32(data);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace

uint8_t crc8_dji(const uint8_t *data, size_t size)
{
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 1U) ? static_cast<uint8_t>((crc >> 1) ^ 0x8C) : static_cast<uint8_t>(crc >> 1);
  }
  return crc;
}

uint16_t crc16_ccitt(const uint8_t *data, size_t size)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408) : static_cast<uint16_t>(crc >> 1);
  }
  return crc;
}

std::vector<uint8_t> encode_command(const CommandFrame &command)
{
  std::vector<uint8_t> payload;
  payload.reserve(kCommandPayloadLength);
  const bool valid = command.enable && std::all_of(
    command.motor_rad_s.begin(), command.motor_rad_s.end(),
    [](float value) { return std::isfinite(value); });
  for (const float value : command.motor_rad_s)
    put_f32(payload, valid ? value : 0.0F);

  std::vector<uint8_t> frame;
  frame.reserve(4 + kCommandPayloadLength + 2);
  frame.push_back(kProtocolVersion);
  frame.push_back(kCommandPayloadLength);
  frame.push_back(static_cast<uint8_t>(FrameType::Command));
  frame.push_back(crc8_dji(frame.data(), 3));
  frame.insert(frame.end(), payload.begin(), payload.end());
  const uint16_t crc = crc16_ccitt(frame.data(), frame.size());
  frame.push_back(static_cast<uint8_t>(crc));
  frame.push_back(static_cast<uint8_t>(crc >> 8));
  return frame;
}

FrameParser::FrameParser(
  FeedbackCallback callback, RawFrameCallback raw_callback,
  RemoteControlCallback remote_callback)
: callback_(std::move(callback)), raw_callback_(std::move(raw_callback)),
  remote_callback_(std::move(remote_callback))
{
}

void FrameParser::reset()
{
  buffer_.clear();
}

void FrameParser::feed(const uint8_t *data, size_t size)
{
  if (data == nullptr || size == 0)
    return;
  buffer_.insert(buffer_.end(), data, data + size);
  if (buffer_.size() > 4096)
  {
    const size_t keep = 1024;
    dropped_bytes_ += buffer_.size() - keep;
    buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(keep));
  }
  consume();
}

void FrameParser::consume()
{
  constexpr uint8_t feedback_id = static_cast<uint8_t>(FrameType::Feedback);
  while (buffer_.size() >= 4)
  {
    if (buffer_[0] != kProtocolVersion)
    {
      buffer_.erase(buffer_.begin());
      ++dropped_bytes_;
      continue;
    }

    const bool is_feedback = buffer_[2] == feedback_id;
    const bool is_remote = buffer_[2] == static_cast<uint8_t>(FrameType::RemoteControl);
    const size_t payload_length = buffer_[1];
    if ((!is_feedback && !is_remote) ||
      (is_feedback && payload_length != kFeedbackPayloadLength) ||
      (is_remote && payload_length != kRemoteControlPayloadLength))
    {
      buffer_.erase(buffer_.begin());
      ++errors_;
      continue;
    }

    if (crc8_dji(buffer_.data(), 3) != buffer_[3])
    {
      buffer_.erase(buffer_.begin());
      ++errors_;
      continue;
    }

    const size_t total_length = 4 + payload_length + 2;
    if (buffer_.size() < total_length)
      return;

    const uint16_t expected_crc = get_u16(buffer_.data() + total_length - 2);
    if (crc16_ccitt(buffer_.data(), total_length - 2) != expected_crc)
    {
      buffer_.erase(buffer_.begin());
      ++errors_;
      continue;
    }

    bool finite = true;
    for (size_t offset = 4; is_feedback && offset < 4 + payload_length; offset += sizeof(float))
      finite = finite && std::isfinite(get_f32(buffer_.data() + offset));
    if (!finite)
    {
      buffer_.erase(buffer_.begin());
      ++errors_;
      continue;
    }

    const std::vector<uint8_t> raw(buffer_.begin(), buffer_.begin() + total_length);
    const std::vector<uint8_t> payload(
      buffer_.begin() + 4, buffer_.begin() + 4 + payload_length);
    if (is_feedback)
    {
      // 保留既有回调和 frames() 的 0x3B 专属语义。
      if (raw_callback_)
        raw_callback_(raw);
      if (callback_)
        callback_(buffer_[0], payload);
      ++frames_;
    }
    else
    {
      RemoteControlFrame remote{};
      if (decode_remote_control(payload, remote) && remote_callback_)
        remote_callback_(remote);
      ++remote_frames_;
    }
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_length);
  }
}

bool decode_remote_control(const std::vector<uint8_t> &payload, RemoteControlFrame &out)
{
  if (payload.size() != kRemoteControlPayloadLength)
    return false;
  out.timestamp_ms = get_u32(payload.data());
  for (size_t i = 0; i < out.channels.size(); ++i)
  {
    const uint16_t bits = get_u16(payload.data() + 4 + 2 * i);
    // 显式符号扩展，不依赖超范围 unsigned -> signed 转换行为。
    out.channels[i] = static_cast<int16_t>(
      bits < 0x8000U ? static_cast<int32_t>(bits) : static_cast<int32_t>(bits) - 65536);
  }
  out.sw_right = payload[12];
  out.sw_left = payload[13];
  return true;
}

bool remote_controls_valid(const RemoteControlFrame &frame)
{
  return frame.sw_right >= 1 && frame.sw_right <= 3 &&
         frame.sw_left >= 1 && frame.sw_left <= 3 &&
         std::all_of(frame.channels.begin(), frame.channels.end(),
    [](int16_t value) { return value >= -700 && value <= 700; });
}

bool decode_feedback(const std::vector<uint8_t> &payload, FeedbackFrame &out)
{
  if (payload.size() != kFeedbackPayloadLength)
    return false;

  size_t offset = 0;
  const auto read_group = [&payload, &offset](std::array<float, 3> &group)
  {
    for (float &value : group)
    {
      value = get_f32(payload.data() + offset);
      if (!std::isfinite(value))
        return false;
      offset += sizeof(float);
    }
    return true;
  };
  return read_group(out.motor_actual) && read_group(out.euler) &&
         read_group(out.gyro) && read_group(out.accel);
}

}  // namespace kiwi_base
