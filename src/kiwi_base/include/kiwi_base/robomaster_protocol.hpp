/**
 * @file robomaster_protocol.hpp
 * @brief RoboMaster C 板 USB CDC 的 0x3A 命令和 0x3B 反馈协议。
 */
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace kiwi_base
{

/** 固定帧头/版本字节。 */
constexpr uint8_t kProtocolVersion = 0x33;
/** 0x3A 三电机命令 payload 长度。 */
constexpr uint8_t kCommandPayloadLength = 12;
/** 0x3B 三电机+欧拉角+六轴 IMU payload 长度。 */
constexpr uint8_t kFeedbackPayloadLength = 48;

/** 本项目实际使用的两个数据 ID；下位机回传只使用 Feedback。 */
enum class FrameType : uint8_t
{
  Command = 0x3A,
  Feedback = 0x3B
};

/** 发送给 C 板的三个电机转子轴目标角速度，单位 rad/s。 */
struct CommandFrame
{
  bool enable{false};
  std::array<float, 3> motor_rad_s{};
};

/**
 * C 板唯一回传帧 0x3B 的 12 个 float32。
 * 顺序：电机实际速度、roll/pitch/yaw、gyro xyz、accel xyz。
 */
struct FeedbackFrame
{
  std::array<float, 3> motor_actual{};
  std::array<float, 3> euler{}; // roll、pitch、yaw
  std::array<float, 3> gyro{}; // x、y、z 角速度
  std::array<float, 3> accel{}; // x、y、z 加速度
};

/** 计算 header 前三个字节的 DJI 反射 CRC8。 */
uint8_t crc8_dji(const uint8_t *data, size_t size);

/** 计算 header+payload 的 DJI 反射 CRC16，线上低字节先传。 */
uint16_t crc16_ccitt(const uint8_t *data, size_t size);

/** 将三个电机轴速度编码为固定 18 字节 0x3A 帧。 */
std::vector<uint8_t> encode_command(const CommandFrame &command);

/**
 * 增量解析 C 板唯一的 0x3B/48 字节反馈帧。
 * 支持半帧、粘包、CRC 错误和错误字节后的重新同步。
 */
class FrameParser
{
public:
  /** 完整反馈回调：参数为版本和不含头尾 CRC 的 48 字节 payload。 */
  using FeedbackCallback = std::function<void(uint8_t version, const std::vector<uint8_t> &payload)>;
  /** 原始反馈回调：参数为已通过长度和 CRC 校验的完整 54 字节帧。 */
  using RawFrameCallback = std::function<void(const std::vector<uint8_t> &frame)>;

  /** 创建反馈解析器。 */
  explicit FrameParser(FeedbackCallback callback = {}, RawFrameCallback raw_callback = {});

  /** 送入任意长度 USB 字节片段。 */
  void feed(const uint8_t *data, size_t size);

  /** 清除未完成半帧，统计计数保留。 */
  void reset();

  /** 返回 CRC、长度、ID 或 payload 校验失败次数。 */
  size_t errors() const noexcept { return errors_; }
  /** 返回成功解析的 0x3B 帧数。 */
  size_t frames() const noexcept { return frames_; }
  /** 返回重同步时丢弃的字节数。 */
  size_t dropped_bytes() const noexcept { return dropped_bytes_; }

private:
  /** 尽可能消费缓存中的完整反馈帧，不等待、不阻塞。 */
  void consume();

  std::vector<uint8_t> buffer_;
  FeedbackCallback callback_{};
  RawFrameCallback raw_callback_{};
  size_t errors_{0};
  size_t frames_{0};
  size_t dropped_bytes_{0};
};

/** 按实际 0x3B 字段偏移解码 12 个 float32。 */
bool decode_feedback(const std::vector<uint8_t> &payload, FeedbackFrame &out);

}  // namespace kiwi_base
