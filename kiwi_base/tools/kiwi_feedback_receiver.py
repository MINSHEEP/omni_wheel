#!/usr/bin/env python3
"""接收并观察 RoboMaster C 板返回的 0x3B 十二浮点反馈帧。

帧格式为 ``0x33/len=48/id=0x3B/crc8 + 12 个小端 float32 + crc16``。
顺序为三个电机实际速度、roll/pitch/yaw、角速度 xyz、线加速度 xyz。
"""

from __future__ import annotations

import argparse
import math
import os
import select
import struct
import sys
import termios
import time
from dataclasses import dataclass
from typing import List, Optional, Sequence


SOF = 0x33
FEEDBACK_ID = 0x3B
PAYLOAD_FORMAT = "<12f"
PAYLOAD_LENGTH = struct.calcsize(PAYLOAD_FORMAT)
FRAME_LENGTH = 4 + PAYLOAD_LENGTH + 2


def crc8_dji(data: bytes) -> int:
    """计算帧头前三字节的 DJI 反射 CRC8。"""
    crc = 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if crc & 1 else (crc >> 1)
    return crc


def crc16_dji(data: bytes) -> int:
    """计算帧头和 payload 的 DJI 反射 CRC16。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if crc & 1 else (crc >> 1)
    return crc


@dataclass(frozen=True)
class FeedbackFrame:
    """一帧已通过长度、CRC 和有限值检查的唯一 0x3B 实机反馈。"""

    motor_actual: tuple[float, float, float]
    euler: tuple[float, float, float]
    gyro: tuple[float, float, float]
    accel: tuple[float, float, float]
    raw_frame: bytes


class FeedbackParser:
    """增量解析 USB 字节流，支持半帧、粘包和错误数据重同步。"""

    def __init__(self) -> None:
        """初始化接收缓存和诊断计数器。"""
        self.buffer = bytearray()
        self.valid_frames = 0
        self.crc_errors = 0
        self.length_errors = 0
        self.payload_errors = 0
        self.dropped_bytes = 0

    def feed(self, data: bytes) -> List[FeedbackFrame]:
        """追加任意长度串口数据，返回本次解析出的所有有效 0x3B 帧。"""
        self.buffer.extend(data)
        if len(self.buffer) > 4096:
            # 限制异常输入的内存占用，同时保留尾部供下一次重同步。
            removed = len(self.buffer) - 1024
            del self.buffer[:removed]
            self.dropped_bytes += removed

        feedback_frames: List[FeedbackFrame] = []
        while len(self.buffer) >= 4:
            sof_index = self.buffer.find(SOF)
            if sof_index < 0:
                self.dropped_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if sof_index > 0:
                del self.buffer[:sof_index]
                self.dropped_bytes += sof_index
            if len(self.buffer) < 4:
                break

            payload_length = self.buffer[1]
            if self.buffer[2] != FEEDBACK_ID or payload_length != PAYLOAD_LENGTH:
                del self.buffer[0]
                self.length_errors += 1
                continue
            if crc8_dji(bytes(self.buffer[:3])) != self.buffer[3]:
                del self.buffer[0]
                self.crc_errors += 1
                continue

            total_length = 4 + payload_length + 2
            if len(self.buffer) < total_length:
                break
            raw_frame = bytes(self.buffer[:total_length])
            expected_crc16 = struct.unpack_from("<H", raw_frame, total_length - 2)[0]
            if crc16_dji(raw_frame[:-2]) != expected_crc16:
                del self.buffer[0]
                self.crc_errors += 1
                continue
            del self.buffer[:total_length]

            values = struct.unpack(PAYLOAD_FORMAT, raw_frame[4:-2])
            if not all(math.isfinite(value) for value in values):
                self.payload_errors += 1
                continue
            feedback_frames.append(
                FeedbackFrame(
                    motor_actual=values[0:3],
                    euler=values[3:6],
                    gyro=values[6:9],
                    accel=values[9:12],
                    raw_frame=raw_frame,
                )
            )
            self.valid_frames += 1
        return feedback_frames


def encode_test_frame(values: Sequence[float]) -> bytes:
    """仅供无硬件自检构造合法帧，不用于向真实下位机发送。"""
    if len(values) != 12 or not all(math.isfinite(float(value)) for value in values):
        raise ValueError("测试帧必须包含十二个有限浮点数")
    payload = struct.pack(PAYLOAD_FORMAT, *(float(value) for value in values))
    header = bytes((SOF, len(payload), FEEDBACK_ID))
    frame_without_crc16 = header + bytes((crc8_dji(header),)) + payload
    return frame_without_crc16 + struct.pack("<H", crc16_dji(frame_without_crc16))


def describe_feedback(feedback: FeedbackFrame) -> str:
    """将一帧反馈格式化为便于串口联调观察的单行文本。"""
    return "motor_actual=[{}] euler_rpy=[{}] gyro_xyz=[{}] accel_xyz=[{}]".format(
        ", ".join(f"{value:.6f}" for value in feedback.motor_actual),
        ", ".join(f"{value:.6f}" for value in feedback.euler),
        ", ".join(f"{value:.6f}" for value in feedback.gyro),
        ", ".join(f"{value:.6f}" for value in feedback.accel),
    )


class PosixSerialReader:
    """pyserial 不可用时使用的 Linux 只读串口后备实现。"""

    _BAUD_RATES = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
        460800: termios.B460800,
        921600: termios.B921600,
    }

    def __init__(self, port: str, baudrate: int, timeout: float) -> None:
        """以 raw、8N1、非阻塞方式打开 Linux TTY 或 USB CDC 设备。"""
        speed = self._BAUD_RATES.get(baudrate)
        if speed is None:
            supported = ", ".join(str(value) for value in sorted(self._BAUD_RATES))
            raise ValueError(f"标准库串口模式不支持 {baudrate} 波特率，可用值: {supported}")
        self.timeout = timeout
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            attributes = termios.tcgetattr(self.fd)
            attributes[0] = 0
            attributes[1] = 0
            attributes[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
            attributes[3] = 0
            attributes[4] = speed
            attributes[5] = speed
            attributes[6][termios.VMIN] = 0
            attributes[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, attributes)
            termios.tcflush(self.fd, termios.TCIFLUSH)
        except Exception:
            os.close(self.fd)
            self.fd = -1
            raise

    @property
    def in_waiting(self) -> int:
        """设备可读时请求一次批量读取，避免逐字节系统调用。"""
        readable, _, _ = select.select([self.fd], [], [], 0.0)
        return 4096 if readable else 0

    def read(self, size: int) -> bytes:
        """在配置的超时内读取最多 size 字节，无数据时返回空字节串。"""
        readable, _, _ = select.select([self.fd], [], [], self.timeout)
        if not readable:
            return b""
        try:
            return os.read(self.fd, max(1, size))
        except BlockingIOError:
            return b""

    def close(self) -> None:
        """关闭串口文件描述符；允许重复调用。"""
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1


def open_serial(port: str, baudrate: int, timeout: float):
    """优先使用 pyserial，缺失时自动使用 Linux 标准库串口读取器。"""
    try:
        import serial  # type: ignore
    except ImportError:
        print("提示: 未检测到 pyserial，使用 Linux 标准库串口模式")
        return PosixSerialReader(port=port, baudrate=baudrate, timeout=timeout)
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


def run_self_test() -> None:
    """验证 golden frame、半帧、粘包、重同步和 CRC 错误处理。"""
    values = tuple(float(value) for value in range(1, 13))
    frame = encode_test_frame(values)
    assert len(frame) == FRAME_LENGTH

    parser = FeedbackParser()
    parsed = parser.feed(b"\x00\xff" + frame[:11])
    assert not parsed
    parsed = parser.feed(frame[11:])
    assert len(parsed) == 1
    assert parsed[0].motor_actual == values[0:3]
    assert parsed[0].euler == values[3:6]
    assert parsed[0].gyro == values[6:9]
    assert parsed[0].accel == values[9:12]
    assert parser.valid_frames == 1
    assert parser.dropped_bytes == 2

    broken = bytearray(frame)
    broken[10] ^= 0x01
    parsed = parser.feed(bytes(broken) + frame)
    assert len(parsed) == 1 and parser.crc_errors == 1
    print("kiwi_feedback_receiver self-test: PASS")
    print("0x3B golden frame: " + frame.hex(" "))


def build_argument_parser() -> argparse.ArgumentParser:
    """构造串口接收测试参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="不打开串口，只运行解析自检")
    parser.add_argument("--port", default="/dev/robomaster_c", help="USB CDC 或串口设备路径")
    parser.add_argument("--baudrate", type=int, default=115200, help="串口波特率；USB CDC 通常忽略")
    parser.add_argument("--read-timeout", type=float, default=0.1, help="单次读取超时，单位秒")
    parser.add_argument("--duration", type=float, default=0.0, help="运行秒数；0 表示持续运行")
    parser.add_argument("--count", type=int, default=0, help="收到指定数量有效帧后退出；0 表示不限制")
    parser.add_argument("--print-hex", action="store_true", help="同时打印每个有效帧的原始十六进制")
    return parser


def receive_feedback(args: argparse.Namespace) -> int:
    """持续读取串口，只输出通过校验的 0x3B 十二浮点反馈。"""
    if args.baudrate <= 0 or args.read_timeout < 0.0 or args.duration < 0.0 or args.count < 0:
        raise ValueError("波特率必须为正，超时、运行时长和帧数不能为负")

    serial_port = open_serial(args.port, args.baudrate, args.read_timeout)
    parser = FeedbackParser()
    started_at = time.monotonic()
    deadline = started_at + args.duration if args.duration > 0.0 else None
    print(
        f"监听 {args.port}，等待 ID=0x{FEEDBACK_ID:02x}、payload={PAYLOAD_LENGTH} B、"
        f"frame={FRAME_LENGTH} B"
    )
    try:
        while deadline is None or time.monotonic() < deadline:
            waiting = getattr(serial_port, "in_waiting", 0)
            data = serial_port.read(waiting or 1)
            if not data:
                continue
            for feedback in parser.feed(data):
                if args.print_hex:
                    print("RX HEX " + feedback.raw_frame.hex(" "))
                print("RX " + describe_feedback(feedback))
                if args.count > 0 and parser.valid_frames >= args.count:
                    return 0
    except KeyboardInterrupt:
        print("\n用户停止接收")
    finally:
        serial_port.close()
        elapsed = max(time.monotonic() - started_at, 1e-9)
        print(
        "统计: valid={} rate={:.2f}Hz length_errors={} crc_errors={} "
                "payload_errors={} dropped_bytes={}".format(
                parser.valid_frames,
                parser.valid_frames / elapsed,
                parser.length_errors,
                parser.crc_errors,
                parser.payload_errors,
                parser.dropped_bytes,
            )
        )
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    """运行无硬件自检或真实串口接收测试。"""
    args = build_argument_parser().parse_args(argv)
    if args.self_test:
        run_self_test()
        return 0
    return receive_feedback(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        raise SystemExit(1)
