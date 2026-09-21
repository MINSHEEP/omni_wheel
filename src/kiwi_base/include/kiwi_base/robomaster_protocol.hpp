/**
 * @file robomaster_protocol.hpp
 * @brief RoboMaster C 板 USB CDC 的 0x3A 命令及异步 0x3B/0x3C 解码。
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
/** 0x3C 时间戳、四个摇杆通道和两个拨杆。 */
constexpr uint8_t kRemoteControlPayloadLength = 14;

/** 0x3A 命令、0x3B 电机/IMU 和 0x3C 遥控类型。 */
enum class FrameType : uint8_t
{
  Command = 0x3A,
  Feedback = 0x3B,
  RemoteControl = 0x3C
};

/** 发送给 C 板的三个电机转子轴目标角速度，单位 rad/s。 */
struct CommandFrame
{
  bool enable{false};
  std::array<float, 3> motor_rad_s{};
};

/**
 * 异步遥测帧 0x3B 的 12 个 float32；它不是 0x3A 的 ACK。
 * 顺序：电机实际速度、roll/pitch/yaw、gyro xyz、accel xyz。
 */
struct FeedbackFrame
{
  std::array<float, 3> motor_actual{};
  std::array<float, 3> euler{}; // roll、pitch、yaw
  std::array<float, 3> gyro{}; // x、y、z 角速度
  std::array<float, 3> accel{}; // x、y、z 加速度
};

/** 当前固件没有 ch[4] 或接收机在线状态；timestamp_ms 是打包时间。 */
struct RemoteControlFrame
{
  uint32_t timestamp_ms{0};
  std::array<int16_t, 4> channels{};
  uint8_t sw_right{0};
  uint8_t sw_left{0};
};

bool decode_remote_control(const std::vector<uint8_t> &payload, RemoteControlFrame &out);
/** 只检查 DT7 数值范围，不能据此判定接收机在线。 */
bool remote_controls_valid(const RemoteControlFrame &frame);

/** 计算 header 前三个字节的 DJI 反射 CRC8。 */
uint8_t crc8_dji(const uint8_t *data, size_t size);

/** 计算 header+payload 的 DJI 反射 CRC16，线上低字节先传。 */
uint16_t crc16_ccitt(const uint8_t *data, size_t size);

/** 将三个电机轴速度编码为固定 18 字节 0x3A 帧。 */
std::vector<uint8_t> encode_command(const CommandFrame &command);

/**
 * 增量解析 0x3B/48 字节反馈和 0x3C/14 字节遥控的混合字节流。
 * 支持半帧、粘包、CRC 错误和错误字节后的重新同步。
 */
class FrameParser
{
public:
  /** 完整反馈回调：参数为版本和不含头尾 CRC 的 48 字节 payload。 */
  using FeedbackCallback = std::function<void(uint8_t version, const std::vector<uint8_t> &payload)>;
  /** 原始反馈回调：参数为已通过长度和 CRC 校验的完整 54 字节帧。 */
  using RawFrameCallback = std::function<void(const std::vector<uint8_t> &frame)>;
  using RemoteControlCallback = std::function<void(const RemoteControlFrame &frame)>;

  /** 创建反馈解析器。 */
  explicit FrameParser(
    FeedbackCallback callback = {}, RawFrameCallback raw_callback = {},
    RemoteControlCallback remote_callback = {});

  /** 送入任意长度 USB 字节片段。 */
  void feed(const uint8_t *data, size_t size);

  /** 清除未完成半帧，统计计数保留。 */
  void reset();

  /** 返回 CRC、长度、ID 或 payload 校验失败次数。 */
  size_t errors() const noexcept { return errors_; }
  /** 返回成功解析的 0x3B 帧数。 */
  size_t frames() const noexcept { return frames_; }
  size_t remote_frames() const noexcept { return remote_frames_; }
  /** 返回重同步时丢弃的字节数。 */
  size_t dropped_bytes() const noexcept { return dropped_bytes_; }

private:
  /** 尽可能消费缓存中的完整反馈帧，不等待、不阻塞。 */
  void consume();

  std::vector<uint8_t> buffer_;
  FeedbackCallback callback_{};
  RawFrameCallback raw_callback_{};
  RemoteControlCallback remote_callback_{};
  size_t errors_{0};
  size_t frames_{0};
  size_t remote_frames_{0};
  size_t dropped_bytes_{0};
};

/** 按 0x3B 字段偏移解码 12 个 float32。 */
bool decode_feedback(const std::vector<uint8_t> &payload, FeedbackFrame &out);

}  // namespace kiwi_base
