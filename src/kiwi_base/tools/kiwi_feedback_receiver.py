#!/usr/bin/env python3
"""离线自检现行 0x3B 十二浮点异步遥测帧解析器。

运行时串口由 ``kiwi_base_node`` 独占并非阻塞接收 0x3B；本工具只允许
``--self-test``，避免与节点争用设备。0x3B 不是 0x3A 命令的 ACK。
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
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
    """一帧已通过长度、CRC 和有限值检查的 0x3B 异步遥测。"""

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
    """将一帧遥测格式化为便于协议联调观察的单行文本。"""
    return "motor_actual=[{}] euler_rpy=[{}] gyro_xyz=[{}] accel_xyz=[{}]".format(
        ", ".join(f"{value:.6f}" for value in feedback.motor_actual),
        ", ".join(f"{value:.6f}" for value in feedback.euler),
        ", ".join(f"{value:.6f}" for value in feedback.gyro),
        ", ".join(f"{value:.6f}" for value in feedback.accel),
    )


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
    """构造只允许离线自检的参数。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="不打开串口，只运行现行 0x3B 帧解析自检",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """仅运行无硬件自检；实时 0x3B 接收由 ROS 节点独占。"""
    args = build_argument_parser().parse_args(argv)
    if not args.self_test:
        print(
            "错误: 0x3B 由 kiwi_base_node 异步接收；本工具只允许 --self-test，"
            "不会打开串口等待回包或与节点争用设备",
            file=sys.stderr,
        )
        return 2
    run_self_test()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"错误: {exc}", file=sys.stderr)
        raise SystemExit(1)
