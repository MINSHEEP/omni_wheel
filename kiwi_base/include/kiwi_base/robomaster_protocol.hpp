/**
 * @file robomaster_protocol.hpp
 * @brief 上位机与 Robomaster C 板之间的 USB CDC 帧编解码和反馈解析接口。
 */
#pragma once
#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace kiwi_base
{
  /*
   * C 型板联调命令协议：SOF/version=0x33，header=[sof,len,id,crc8]，
   * id=0x3A、payload 为三个 little-endian float32 电机轴 rad/s，尾部 CRC16。
   * 实机反馈使用 id=0x3B、payload 为十二个 little-endian float32；旧反馈 ID 暂留兼容。
   */
  constexpr uint8_t kProtocolVersion = 0x33;
  constexpr uint8_t kMotorCommandPayloadLength = 12;
  constexpr uint8_t kMotorImuFeedbackPayloadLength = 48;
  /** 当前协议支持的命令、实机组合反馈及旧适配反馈数据 ID。 */
  enum class FrameType : uint8_t
  {
    /** 用户确认的三电机速度命令 ID。 */
    Command = 0x3A,
    /** 实机确认的三电机实际速度、欧拉角、角速度和加速度组合反馈。 */
    MotorImuFeedback = 0x3B,
    WheelState = 0x80,
    ImuState = 0x81,
    PowerState = 0x82
  };
  /** 主机发送的三路电机轴目标角速度；线上只编码 motor_rad_s 三个 float。 */
  struct CommandFrame
  {
    /* 以下字段只供主机诊断/安全门控使用，不会写入本次 12 字节 payload。 */
    uint16_t seq{0};
    uint32_t timestamp_ms{0};
    bool enable{false};
    std::array<float, 3> motor_rad_s{};
  };
  /** C 板 0x3B 组合反馈；十二个 float 的顺序由实机抓包确认。 */
  struct MotorImuFeedbackFrame
  {
    std::array<float, 3> motor_actual{};
    std::array<float, 3> euler{};
    std::array<float, 3> gyro{};
    std::array<float, 3> accel{};
  };
  /** C 板反馈的轮位置、轮速度、原始电流、电压和故障状态。 */
  struct WheelStateFrame
  {
    uint16_t state_seq{0};
    uint16_t last_cmd_seq{0};
    uint32_t timestamp_ms{0};
    std::array<float, 3> position{};
    std::array<float, 3> velocity{};
    std::array<int16_t, 3> current_raw{};
    uint16_t battery_mv{0};
    uint32_t fault_bits{0};
    uint16_t flags{0};
  };
  /** C 板 BMI088 的角速度、线加速度和温度快照。 */
  struct ImuStateFrame
  {
    uint16_t seq{0};
    uint32_t timestamp_ms{0};
    std::array<float, 3> gyro{};
    std::array<float, 3> accel{};
    float temperature{0.0f};
  };
  /** 可选的独立电源状态反馈；当前固件将电压嵌入轮状态。 */
  struct PowerStateFrame
  {
    uint16_t seq{0};
    uint32_t timestamp_ms{0};
    uint16_t battery_mv{0};
    uint16_t flags{0};
  };

  /** 计算联调协议使用的 DJI 反射 CRC8（覆盖 header 前 3 字节）。 */
  uint8_t crc8_dji(const uint8_t *data, size_t size);

  /** 计算联调协议使用的反射 CRC16（覆盖 header+payload，尾部低字节先行）。 */
  uint16_t crc16_ccitt(const uint8_t *data, size_t size);

  /** 将三路电机轴速度编码为 18 字节联调帧；enable=false 时线上强制发送零速。 */
  std::vector<uint8_t> encode_command(const CommandFrame &command);

  /**
   * @brief 增量解析 USB 字节流，并在 CRC 与字段校验通过后分发完整反馈帧。
   *
   * 解析器允许半帧、粘包和错误字节输入，内部缓存有上限，适合在非阻塞串口回调中使用。
   */
  class FrameParser
  {
  public:
    /** 完整帧回调：参数依次为协议版本、数据 ID 和不含头尾 CRC 的 payload。 */
    using FrameCallback = std::function<void(uint8_t version, FrameType type, const std::vector<uint8_t> &payload)>;
    /** 原始帧回调：参数是已通过长度、CRC 和有限值校验的完整线上帧。 */
    using RawFrameCallback = std::function<void(const std::vector<uint8_t> &frame)>;
    /** 创建解析器；两个回调都在 feed() 调用线程内同步执行。 */
    explicit FrameParser(FrameCallback callback = {}, RawFrameCallback raw_callback = {});

    /** 送入任意长度的 USB 字节片段，支持半帧和粘包。 */
    void feed(const uint8_t *data, size_t size);

    /** 清除未完成帧，通常在 USB 重连或更换设备时调用。 */
    void reset();

    /** 返回累计 CRC、长度或非法浮点校验失败次数。 */
    size_t crc_errors() const noexcept { return crc_errors_; }

    /** 返回累计成功回调的完整帧数。 */
    size_t frames() const noexcept { return frames_; }

    /** 返回重同步时丢弃的字节数。 */
    size_t dropped_bytes() const noexcept { return dropped_bytes_; }

  private:
    /** 从缓存中尽可能消费完整帧；不阻塞、不等待更多字节。 */
    void consume();
    std::vector<uint8_t> buffer_;
    FrameCallback callback_{};
    RawFrameCallback raw_callback_{};
    size_t crc_errors_{0}, frames_{0}, dropped_bytes_{0};
  };
  /** 按临时字段偏移解码轮状态，长度或浮点非法时返回 false。 */
  bool decode_wheel_state(const std::vector<uint8_t> &payload, WheelStateFrame &out);

  /** 按实机确认的十二浮点顺序解码 0x3B 组合反馈。 */
  bool decode_motor_imu_feedback(
      const std::vector<uint8_t> &payload, MotorImuFeedbackFrame &out);

  /** 按临时字段偏移解码 IMU 状态，长度或浮点非法时返回 false。 */
  bool decode_imu_state(const std::vector<uint8_t> &payload, ImuStateFrame &out);

  /** 按临时字段偏移解码电源状态。 */
  bool decode_power_state(const std::vector<uint8_t> &payload, PowerStateFrame &out);
} // namespace kiwi_base
