#!/usr/bin/env python3
"""Kiwi 三轮底盘 USB 串口测试工具。

当前联调命令协议为 0x33/len/0x3A/CRC8 + 三个电机轴 rad/s float32 + CRC16。
也支持 ``--raw-hex`` 原始字节模式，便于联调反馈帧时不猜测未知字段。
默认不使能电机；未传 ``--enable`` 时线上发送三个零速值。
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple


SOF = 0x33
CMD_MOTOR_SPEED = 0x3A
STATE_WHEEL = 0x80
STATE_IMU = 0x81


def crc8_dji(data: bytes) -> int:
    """计算联调协议的 DJI 反射 CRC8（init=0xff，poly=0x8c）。"""
    crc = 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if crc & 1 else (crc >> 1)
    return crc


def crc16_dji(data: bytes) -> int:
    """计算联调协议的反射 CRC16（init=0xffff，poly=0x8408）。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if crc & 1 else (crc >> 1)
    return crc


def kiwi_inverse(
    vx: float,
    vy: float,
    wz: float,
    radius_m: float = 0.1015,
    center_distance_m: float = 0.32256,
    signs: Sequence[int] = (1, 1, 1),
) -> Tuple[float, float, float]:
    """按三轮 120 度对称初始假设，把机体 Twist 逆解为轮输出轴 rad/s。

    轮 1 位于 base_link 的 +y，径向角取 90 度，轮 2/3 依次转 120 度；
    对应滚动切向角为 0/120/240 度。轮序、切向方向和 sign 必须在实车低速点动后修正。
    """
    values = (vx, vy, wz, radius_m, center_distance_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("速度和几何参数必须是有限数")
    if radius_m <= 0.0 or center_distance_m <= 0.0:
        raise ValueError("轮半径和轮心距必须为正")
    if len(signs) != 3 or any(sign not in (-1, 1) for sign in signs):
        raise ValueError("signs 必须是三个 +1 或 -1")

    result: List[float] = []
    for index, sign in enumerate(signs):
        radial = math.radians(90.0 + 120.0 * index)
        tangent = math.radians(120.0 * index)
        x = center_distance_m * math.cos(radial)
        y = center_distance_m * math.sin(radial)
        tx = math.cos(tangent)
        ty = math.sin(tangent)
        result.append(sign / radius_m * (tx * vx + ty * vy + (x * ty - y * tx) * wz))
    return result[0], result[1], result[2]


def wheel_to_motor(
    wheel_rad_s: Sequence[float], reduction_ratio: float = 19.0
) -> Tuple[float, float, float]:
    """把轮输出轴 rad/s 转为联调帧所需的电机轴 rad/s。"""
    if len(wheel_rad_s) != 3 or not math.isfinite(reduction_ratio) or reduction_ratio <= 0.0:
        raise ValueError("需要三个轮速且 reduction_ratio 必须为正数")
    if not all(math.isfinite(float(value)) for value in wheel_rad_s):
        raise ValueError("轮速必须是有限数")
    return tuple(float(value) * reduction_ratio for value in wheel_rad_s)  # type: ignore[return-value]


def encode_motor_command(
    seq: int,
    motor_rad_s: Sequence[float],
    enable: bool,
    timestamp_ms: Optional[int] = None,
) -> bytes:
    """编码 18 字节联调帧；seq/timestamp 仅保留 API 兼容，不在线传输。"""
    if len(motor_rad_s) != 3:
        raise ValueError("必须提供三个电机轴速度")
    if not all(math.isfinite(float(value)) for value in motor_rad_s):
        raise ValueError("电机轴速度必须是有限数")
    # 协议没有 enable 字段；禁能由零速和 MCU watchdog 共同保证。
    values = tuple(float(value) if enable else 0.0 for value in motor_rad_s)
    payload = struct.pack("<3f", *values)
    header = bytes((SOF, len(payload), CMD_MOTOR_SPEED))
    frame_without_crc16 = header + bytes((crc8_dji(header),)) + payload
    return frame_without_crc16 + struct.pack("<H", crc16_dji(frame_without_crc16))


def encode_wheel_command(
    seq: int,
    motor_rad_s: Sequence[float],
    enable: bool,
    timestamp_ms: Optional[int] = None,
) -> bytes:
    """兼容旧调用名；参数现在必须是电机轴 rad/s，而不是轮输出轴速度。"""
    return encode_motor_command(seq, motor_rad_s, enable, timestamp_ms)


@dataclass
class FeedbackFrame:
    """当前反馈适配格式解析出的完整帧。"""

    message_id: int
    payload: bytes


class ProvisionalParser:
    """解析当前 0x33 反馈帧，并在协议未知时保留原始字节输出能力。"""

    _PAYLOAD_LENGTHS = {STATE_WHEEL: 46, STATE_IMU: 34}

    def __init__(self) -> None:
        """初始化半帧缓存和联调统计计数器。"""
        self.buffer = bytearray()
        self.crc_errors = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> List[FeedbackFrame]:
        """追加任意长度数据并返回本次解析出的完整帧。"""
        self.buffer.extend(data)
        if len(self.buffer) > 4096:
            # 异常输入不能无限占用内存，只保留最后一段用于重新同步。
            del self.buffer[:-1024]
        frames: List[FeedbackFrame] = []
        while len(self.buffer) >= 4:
            if self.buffer[0] != SOF:
                del self.buffer[0]
                self.dropped_bytes += 1
                continue
            payload_len = self.buffer[1]
            message_id = self.buffer[2]
            expected_payload_len = self._PAYLOAD_LENGTHS.get(message_id)
            if expected_payload_len is None or payload_len != expected_payload_len:
                del self.buffer[0]
                self.dropped_bytes += 1
                continue
            if crc8_dji(bytes(self.buffer[:3])) != self.buffer[3]:
                del self.buffer[0]
                self.crc_errors += 1
                continue
            total = 4 + payload_len + 2
            if len(self.buffer) < total:
                break
            frame = bytes(self.buffer[:total])
            expected_crc = struct.unpack_from("<H", frame, total - 2)[0]
            if crc16_dji(frame[:-2]) != expected_crc:
                del self.buffer[0]
                self.crc_errors += 1
                continue
            frames.append(FeedbackFrame(message_id, frame[4:-2]))
            del self.buffer[:total]
        return frames


def describe_feedback(frame: FeedbackFrame) -> str:
    """把当前轮速/IMU反馈转换为便于台架观察的一行中文文本。"""
    if frame.message_id == STATE_WHEEL and len(frame.payload) == 46:
        values = struct.unpack("<HHI3f3f3hHIH", frame.payload)
        position = values[3:6]
        velocity = values[6:9]
        battery_mv = values[12]
        fault_bits = values[13]
        flags = values[14]
        return (
            "WHEEL pos(rad)={} vel(rad/s)={} battery={:.3f}V fault=0x{:08x} flags=0x{:04x}"
            .format(
                ",".join(f"{value:.4f}" for value in position),
                ",".join(f"{value:.4f}" for value in velocity),
                battery_mv / 1000.0,
                fault_bits,
                flags,
            )
        )
    if frame.message_id == STATE_IMU and len(frame.payload) == 34:
        values = struct.unpack("<HI7f", frame.payload)
        gyro = values[2:5]
        accel = values[5:8]
        return "IMU gyro(rad/s)={} accel(m/s2)={} temp={:.2f}C".format(
            ",".join(f"{value:.4f}" for value in gyro),
            ",".join(f"{value:.4f}" for value in accel),
            values[8],
        )
    return f"未知反馈 msg_id=0x{frame.message_id:02x} payload={frame.payload.hex(' ')}"


def parse_hex_bytes(text: str) -> bytes:
    """解析用户提供的十六进制原始帧，允许空格、冒号和逗号。"""
    cleaned = text.replace("0x", "").replace("0X", "")
    for separator in (" ", "\t", "\n", ":", ",", "-"):
        cleaned = cleaned.replace(separator, "")
    if not cleaned or len(cleaned) % 2:
        raise ValueError("原始十六进制必须包含偶数个非空字符")
    try:
        return bytes.fromhex(cleaned)
    except ValueError as exc:
        raise ValueError(f"无法解析原始十六进制: {exc}") from exc


def run_self_test() -> None:
    """运行无硬件自检：CRC、18 字节命令向量、运动学几何和反馈解析。"""
    golden = bytes.fromhex(
        "33 0c 3a 65 00 00 00 00 00 00 00 00 00 00 00 00 02 fd"
    )
    assert encode_motor_command(1, (0.0, 0.0, 0.0), True, timestamp_ms=0) == golden
    assert len(encode_motor_command(1, (1.0, -2.0, 3.0), True)) == 18
    assert encode_motor_command(1, (1.0, -2.0, 3.0), True).hex(" ") == (
        "33 0c 3a 65 00 00 80 3f 00 00 00 c0 00 00 40 40 64 8e"
    )
    assert encode_motor_command(1, (1.0, -2.0, 3.0), False)[4:-2] == bytes(12)
    wheels = kiwi_inverse(0.0, 0.0, 1.0)
    expected = -0.32256 / 0.1015
    assert all(abs(value - expected) < 1e-9 for value in wheels)
    assert wheel_to_motor(wheels) == tuple(value * 19.0 for value in wheels)
    parser = ProvisionalParser()
    wheel_payload = struct.pack(
        "<HHI3f3f3hHIH", 1, 1, 2, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0, 0, 0, 24000, 0, 0x0C
    )
    wheel_header = bytes((SOF, len(wheel_payload), STATE_WHEEL))
    wheel_frame = wheel_header + bytes((crc8_dji(wheel_header),)) + wheel_payload
    wheel_frame += struct.pack("<H", crc16_dji(wheel_frame))
    parsed = parser.feed(wheel_frame[:7]) + parser.feed(wheel_frame[7:])
    assert len(parsed) == 1 and "WHEEL" in describe_feedback(parsed[0])
    print("kiwi_serial_test self-test: PASS")


def open_serial(port: str, baudrate: int, timeout: float):
    """延迟导入 pyserial 并打开 CDC 设备；USB CDC 实际不使用波特率。"""
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError("缺少 pyserial，请安装 python3-serial 后重试") from exc
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout, write_timeout=timeout)


def build_argument_parser() -> argparse.ArgumentParser:
    """构造命令行参数解析器，默认保持电机禁能。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="只运行无硬件自检")
    parser.add_argument("--port", default="/dev/robomaster_c", help="USB CDC 设备路径")
    parser.add_argument("--baudrate", type=int, default=115200, help="兼容串口 API；CDC 不使用该值")
    parser.add_argument("--rate", type=float, default=20.0, help="命令发送频率 Hz")
    parser.add_argument("--duration", type=float, default=1.0, help="发送持续时间 s；0 表示只发一帧")
    parser.add_argument("--vx", type=float, default=0.0, help="机体前向速度 m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="机体左向速度 m/s")
    parser.add_argument("--wz", type=float, default=0.0, help="机体逆时针角速度 rad/s")
    parser.add_argument("--wheel-rad-s", nargs=3, type=float, metavar=("W1", "W2", "W3"), help="给三个轮输出轴 rad/s，随后乘减速比")
    parser.add_argument("--motor-rad-s", nargs=3, type=float, metavar=("M1", "M2", "M3"), help="直接给协议所需的三个电机轴 rad/s")
    parser.add_argument("--radius-m", type=float, default=0.1015, help="轮半径 m")
    parser.add_argument("--center-distance-m", type=float, default=0.32256, help="底盘中心到轮心距离 m")
    parser.add_argument("--wheel-sign", nargs=3, type=int, choices=(-1, 1), metavar=("S1", "S2", "S3"), help="三个轮的物理正方向符号")
    parser.add_argument("--reduction-ratio", type=float, default=19.0, help="轮输出轴到电机轴的减速比")
    parser.add_argument("--enable", action="store_true", help="允许发送非零目标；未指定时强制零速")
    parser.add_argument("--raw-hex", help="原始十六进制帧；提供后跳过命令编码器")
    parser.add_argument("--dry-run", action="store_true", help="只打印计算结果，不打开串口")
    parser.add_argument("--print-hex", action="store_true", help="打印每次发送的十六进制")
    parser.add_argument("--read-timeout", type=float, default=0.02, help="每次发送后的读取窗口 s")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """执行一次安全的电机速度发送循环，并打印可识别的反馈帧。"""
    args = build_argument_parser().parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    if args.rate <= 0.0 or args.duration < 0.0:
        raise SystemExit("--rate 必须为正，--duration 不能为负")
    if args.raw_hex and (args.wheel_rad_s or args.motor_rad_s):
        raise SystemExit("--raw-hex 不能与速度参数同时使用")
    if args.wheel_rad_s and args.motor_rad_s:
        raise SystemExit("--wheel-rad-s 与 --motor-rad-s 不能同时使用")
    if args.raw_hex:
        frame = parse_hex_bytes(args.raw_hex)
        motor_targets = None
    else:
        if args.motor_rad_s:
            motor_targets = tuple(args.motor_rad_s)
        else:
            wheels = tuple(args.wheel_rad_s) if args.wheel_rad_s else kiwi_inverse(
                args.vx, args.vy, args.wz, args.radius_m, args.center_distance_m,
                tuple(args.wheel_sign) if args.wheel_sign else (1, 1, 1)
            )
            motor_targets = wheel_to_motor(wheels, args.reduction_ratio)
        frame = None
    if motor_targets is not None:
        print("目标电机轴速度 rad/s: " + ", ".join(f"{value:.6f}" for value in motor_targets))
    if args.raw_hex:
        print("原始模式：不猜测反馈字段")
    if args.dry_run:
        if frame is None:
            frame = encode_motor_command(0, motor_targets or (0.0, 0.0, 0.0), args.enable, 0)
        print("发送帧: " + frame.hex(" "))
        return 0

    serial_port = open_serial(args.port, args.baudrate, args.read_timeout)
    parser = ProvisionalParser()
    sequence = 0
    period = 1.0 / args.rate
    deadline = time.monotonic() + args.duration
    try:
        while True:
            now = time.monotonic()
            if args.duration > 0.0 and now >= deadline:
                break
            if args.raw_hex:
                outgoing = frame or b""
            else:
                outgoing = encode_motor_command(sequence, motor_targets or (0.0, 0.0, 0.0), args.enable)
            serial_port.write(outgoing)
            serial_port.flush()
            if args.print_hex:
                print("TX " + outgoing.hex(" "))
            read_deadline = time.monotonic() + max(0.0, args.read_timeout)
            while time.monotonic() < read_deadline:
                waiting = getattr(serial_port, "in_waiting", 0)
                data = serial_port.read(waiting or 1)
                if not data:
                    break
                for feedback in parser.feed(data):
                    print("RX " + describe_feedback(feedback))
            sequence = (sequence + 1) & 0xFFFF
            if args.duration == 0.0:
                break
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print("收到 Ctrl-C，发送禁能零速帧")
    finally:
        if not args.raw_hex:
            stop = encode_motor_command(sequence, (0.0, 0.0, 0.0), False, timestamp_ms=0)
            try:
                serial_port.write(stop)
                serial_port.flush()
            except Exception as exc:  # 关闭阶段不能掩盖原始异常
                print(f"发送停止帧失败: {exc}", file=sys.stderr)
        serial_port.close()
    print(f"解析统计: crc_errors={parser.crc_errors}, dropped_bytes={parser.dropped_bytes}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
