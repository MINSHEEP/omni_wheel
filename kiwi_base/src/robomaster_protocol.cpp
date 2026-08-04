/**
 * @file robomaster_protocol.cpp
 * @brief 实现 USB CDC 命令帧编码、CRC 校验和反馈帧解析。
 */
#include "kiwi_base/robomaster_protocol.hpp"
#include <algorithm>
#include <cstring>
#include <cmath>
namespace kiwi_base
{
    namespace
    {
        /** 以小端序追加无符号 32 位整数。 */
        void put_u32(std::vector<uint8_t> &b, uint32_t x)
        {
            for (int i = 0; i < 4; ++i)
                b.push_back(static_cast<uint8_t>(x >> (8 * i)));
        }
        /** 按 IEEE-754 float32 的位表示以小端序追加浮点数。 */
        void put_f32(std::vector<uint8_t> &b, float x)
        {
            uint32_t u;
            std::memcpy(&u, &x, 4);
            put_u32(b, u);
        }
        /** 从至少 2 字节的缓冲区读取小端序 16 位整数。 */
        uint16_t get_u16(const uint8_t *p) { return uint16_t(p[0]) | uint16_t(p[1]) << 8; }

        /** 从至少 4 字节的缓冲区读取小端序 32 位整数。 */
        uint32_t get_u32(const uint8_t *p) { return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24); }

        /** 将小端序 32 位位模式恢复为 IEEE-754 float32。 */
        float get_f32(const uint8_t *p)
        {
            float x;
            uint32_t u = get_u32(p);
            std::memcpy(&x, &u, 4);
            return x;
        }
    }
    /** DJI 反射 CRC8：初值 0xff，多项式 0x8c；临时协议用于 header 前 3 字节。 */
    uint8_t crc8_dji(const uint8_t *d, size_t n)
    {
        uint8_t crc = 0xff;
        for (size_t i = 0; i < n; ++i)
        {
            crc ^= d[i];
            for (int j = 0; j < 8; ++j)
                crc = (crc & 1) ? uint8_t((crc >> 1) ^ 0x8c) : uint8_t(crc >> 1);
        }
        return crc;
    }
    /**
     * DJI 反射 CRC16：初值 0xffff、多项式 0x8408，结果在线上低字节先行。
     * 名称虽保留 ccitt，不等同于 CRC-16/CCITT-FALSE。
     */
    uint16_t crc16_ccitt(const uint8_t *data, size_t n)
    {
        uint16_t crc = 0xffff;
        for (size_t i = 0; i < n; ++i)
        {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j)
                crc = (crc & 1) ? uint16_t((crc >> 1) ^ 0x8408) : uint16_t(crc >> 1);
        }
        return crc;
    }
    /** 编码 18 字节电机速度帧：4B header、12B（三个 float32）、2B CRC16。 */
    std::vector<uint8_t> encode_command(const CommandFrame &c)
    {
        std::vector<uint8_t> payload;
        payload.reserve(kMotorCommandPayloadLength);
        const bool valid_command = c.enable && std::all_of(
            c.motor_rad_s.begin(), c.motor_rad_s.end(),
            [](float value) { return std::isfinite(value); });
        for (float x : c.motor_rad_s)
        {
            // 任一路非法时整帧归零，不能让其余电机继续执行部分运动命令。
            put_f32(payload, valid_command ? x : 0.0F);
        }
        std::vector<uint8_t> out;
        out.reserve(4 + kMotorCommandPayloadLength + 2);
        out.push_back(kProtocolVersion);
        out.push_back(kMotorCommandPayloadLength);
        out.push_back(static_cast<uint8_t>(FrameType::Command));
        out.push_back(crc8_dji(out.data(), 3));
        for (const auto byte : payload)
            out.push_back(byte);
        const uint16_t crc = crc16_ccitt(out.data(), out.size());
        out.push_back(uint8_t(crc));
        out.push_back(uint8_t(crc >> 8));
        return out;
    }
    /** 保存完整帧回调；解析在调用 feed() 的线程中同步执行。 */
    FrameParser::FrameParser(FrameCallback cb) : callback_(std::move(cb)) {}

    /** 清除半帧缓存，计数器保留用于诊断。 */
    void FrameParser::reset() { buffer_.clear(); }

    /** 接受任意长度字节片段，支持 USB 分片、粘包和一次多帧。 */
    void FrameParser::feed(const uint8_t *d, size_t n)
    {
        if (d == nullptr || n == 0)
            return;
        buffer_.insert(buffer_.end(), d, d + n);
        // 异常输入不能无限占用主机内存；保留尾部用于下一次重同步。
        if (buffer_.size() > 4096)
        {
            const size_t keep = 1024;
            dropped_bytes_ += buffer_.size() - keep;
            buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(keep));
        }
        consume();
    }
    /** 丢弃未知 SOF/类型、错误长度或 CRC，并尽可能从后续字节重同步。 */
    void FrameParser::consume()
    {
        while (buffer_.size() >= 4)
        {
            if (buffer_[0] != kProtocolVersion)
            {
                buffer_.erase(buffer_.begin());
                ++dropped_bytes_;
                continue;
            }
            const uint8_t len = buffer_[1];
            const auto type = static_cast<FrameType>(buffer_[2]);
            size_t expected_len = 0;
            switch (type)
            {
            case FrameType::Command:
                expected_len = kMotorCommandPayloadLength;
                break;
            case FrameType::WheelState:
                expected_len = 46;
                break;
            case FrameType::ImuState:
                expected_len = 34;
                break;
            case FrameType::PowerState:
                expected_len = 10;
                break;
            default:
                buffer_.erase(buffer_.begin());
                ++dropped_bytes_;
                continue;
            }
            if (len != expected_len || crc8_dji(buffer_.data(), 3) != buffer_[3])
            {
                buffer_.erase(buffer_.begin());
                ++crc_errors_;
                continue;
            }
            const size_t total = 4 + len + 2;
            if (buffer_.size() < total)
                return;
            const uint16_t expected = get_u16(buffer_.data() + total - 2);
            const uint16_t actual = crc16_ccitt(buffer_.data(), 4 + len);
            if (actual != expected)
            {
                ++crc_errors_;
                buffer_.erase(buffer_.begin());
                continue;
            }
            if (type == FrameType::Command)
            {
                // 线上三个速度必须是有限 float；非法值不能触发 MCU watchdog 刷新。
                bool finite = true;
                for (size_t offset = 4; offset < 4 + kMotorCommandPayloadLength; offset += 4)
                {
                    const float value = get_f32(buffer_.data() + offset);
                    finite = finite && std::isfinite(value);
                }
                if (!finite)
                {
                    ++crc_errors_;
                    buffer_.erase(buffer_.begin());
                    continue;
                }
            }
            if (callback_)
                callback_(buffer_[0], type, std::vector<uint8_t>(buffer_.begin() + 4, buffer_.begin() + 4 + len));
            ++frames_;
            buffer_.erase(buffer_.begin(), buffer_.begin() + total);
        }
    }
    /** 解码 46 字节轮状态；位置/速度均为轮输出轴 rad、rad/s。 */
    bool decode_wheel_state(const std::vector<uint8_t> &p, WheelStateFrame &o)
    {
        if (p.size() != 46)
            return false;
        size_t i = 0;
        o.state_seq = get_u16(p.data());
        i = 2;
        o.last_cmd_seq = get_u16(p.data() + i);
        i = 4;
        o.timestamp_ms = get_u32(p.data() + i);
        i += 4;
        for (auto &x : o.position)
        {
            x = get_f32(p.data() + i);
            if (!std::isfinite(x))
                return false;
            i += 4;
        }
        for (auto &x : o.velocity)
        {
            x = get_f32(p.data() + i);
            if (!std::isfinite(x))
                return false;
            i += 4;
        }
        for (auto &x : o.current_raw)
        {
            x = static_cast<int16_t>(get_u16(p.data() + i));
            i += 2;
        }
        o.battery_mv = get_u16(p.data() + i);
        i += 2;
        o.fault_bits = get_u32(p.data() + i);
        i += 4;
        o.flags = get_u16(p.data() + i);
        return true;
    }
    /** 解码 34 字节 IMU 状态；角速度 rad/s、加速度 m/s^2。 */
    bool decode_imu_state(const std::vector<uint8_t> &p, ImuStateFrame &o)
    {
        if (p.size() != 34)
            return false;
        size_t i = 0;
        o.seq = get_u16(p.data());
        i = 2;
        o.timestamp_ms = get_u32(p.data() + i);
        i = 6;
        for (auto &x : o.gyro)
        {
            x = get_f32(p.data() + i);
            if (!std::isfinite(x))
                return false;
            i += 4;
        }
        for (auto &x : o.accel)
        {
            x = get_f32(p.data() + i);
            if (!std::isfinite(x))
                return false;
            i += 4;
        }
        o.temperature = get_f32(p.data() + i);
        return std::isfinite(o.temperature);
    }
    /** 解码 10 字节电源状态；当前仅作为未来协议扩展保留。 */
    bool decode_power_state(const std::vector<uint8_t> &p, PowerStateFrame &o)
    {
        if (p.size() != 10)
            return false;
        o.seq = get_u16(p.data());
        o.timestamp_ms = get_u32(p.data() + 2);
        o.battery_mv = get_u16(p.data() + 6);
        o.flags = get_u16(p.data() + 8);
        return true;
    }
} // namespace kiwi_base
