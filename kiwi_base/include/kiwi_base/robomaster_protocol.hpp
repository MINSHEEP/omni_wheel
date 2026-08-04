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
   * 反馈帧仍沿用当前 0x80/0x81/0x82 适配格式；正式协议若调整，只替换本 codec 层。
   */
  constexpr uint8_t kProtocolVersion = 0x33;
  constexpr uint8_t kMotorCommandPayloadLength = 12;
  enum class FrameType : uint8_t
  {
    /** 用户确认的三电机速度命令 ID。 */
    Command = 0x3A,
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

  class FrameParser
  {
  public:
    using FrameCallback = std::function<void(uint8_t version, FrameType type, const std::vector<uint8_t> &payload)>;
    /** 创建解析器；回调在完整帧通过校验后同步执行。 */
    explicit FrameParser(FrameCallback callback = {});

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
    size_t crc_errors_{0}, frames_{0}, dropped_bytes_{0};
  };
  /** 按临时字段偏移解码轮状态，长度或浮点非法时返回 false。 */
  bool decode_wheel_state(const std::vector<uint8_t> &payload, WheelStateFrame &out);

  /** 按临时字段偏移解码 IMU 状态，长度或浮点非法时返回 false。 */
  bool decode_imu_state(const std::vector<uint8_t> &payload, ImuStateFrame &out);

  /** 按临时字段偏移解码电源状态。 */
  bool decode_power_state(const std::vector<uint8_t> &payload, PowerStateFrame &out);
} // namespace kiwi_base
