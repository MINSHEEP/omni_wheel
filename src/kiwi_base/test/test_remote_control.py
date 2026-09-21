"""PTY 集成测试：混合反馈、遥控超时、恢复和命令链互不干扰。"""

import os
import pty
import struct
import subprocess
import time

import rclpy
from kiwi_base.msg import FeedbackDebug, RemoteControl

from test_cmd_vel_watchdog import (
    PACKAGE,
    crc8_dji,
    crc16_ccitt,
    encode_feedback,
    find_node_executable,
    read_command_frames,
)


def encode_remote(timestamp=123, channels=(-660, 0, 660, -1), switches=(3, 1)):
    """按固件 uint32 + int16[4] + uint8[2] 布局构造帧。"""
    header = bytes((0x33, 14, 0x3C))
    frame = header + bytes((crc8_dji(header),))
    frame += struct.pack("<I4h2B", timestamp, *channels, *switches)
    return frame + struct.pack("<H", crc16_ccitt(frame))


def test_remote_stream_timeout_and_recovery(monkeypatch, tmp_path):
    """持续 0x3B 和坏 CRC 均不能刷新遥控计时器；0x3C 不生成里程计。"""
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    monkeypatch.setenv(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        str(PACKAGE / "test" / "fastdds_shm_profile.xml"),
    )
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    os.set_blocking(master, False)
    os.close(slave)
    command = [
        str(find_node_executable()), "--ros-args", "--params-file",
        str(PACKAGE / "config" / "kiwi_base.yaml"),
        "-p", f"device:={device}",
    ]
    node = None
    process = None
    try:
        # 文件日志避免管道填满后阻塞被测节点。
        with (tmp_path / "node.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=log)
            rclpy.init()
            node = rclpy.create_node("kiwi_remote_test")
            messages, feedback, commands = [], [], []
            serial_buffer = b""
            node.create_subscription(
                RemoteControl, "/chassis/remote_control", messages.append, 10,
            )
            node.create_subscription(
                FeedbackDebug, "/chassis/feedback/raw", feedback.append, 10,
            )

            def spin_until(predicate, timeout=3.0, packet=None):
                """等待可观测结果，可同时持续模拟固件每 20 ms 上报。"""
                nonlocal serial_buffer
                deadline = time.monotonic() + timeout
                while not predicate():
                    assert process.poll() is None, "底盘节点异常退出"
                    assert time.monotonic() < deadline, "等待遥控状态超时"
                    if packet is not None:
                        os.write(master, packet)
                    rclpy.spin_once(node, timeout_sec=0.02)
                    frames, serial_buffer = read_command_frames(master, serial_buffer)
                    commands.extend(frames)
                    time.sleep(0.01)

            spin_until(lambda: bool(messages))
            assert not messages[-1].frame_fresh
            assert not messages[-1].controls_valid
            rc = encode_remote()
            # 只有 0x3C 时不能触发旧 0x3B 回调。
            spin_until(lambda: messages[-1].frame_fresh, packet=rc)
            assert not feedback
            msg = messages[-1]
            assert list(msg.channels) == [-660, 0, 660, -1]
            assert (msg.sw_right, msg.sw_left) == (3, 1)
            assert msg.mcu_timestamp_ms == 123
            assert msg.controls_valid

            # 先排空发现期间可能已发出的其余合法 RC，固定最后的接收时间。
            settle_deadline = time.monotonic() + 0.10
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
            msg = messages[-1]

            telemetry = encode_feedback([0.0] * 11 + [9.81])
            corrupt = bytearray(encode_remote(456))
            corrupt[-1] ^= 1
            before_commands = len(commands)
            spin_until(
                lambda: not messages[-1].frame_fresh and bool(feedback),
                packet=telemetry + corrupt,
            )
            assert messages[-1].header.stamp == msg.header.stamp
            assert messages[-1].mcu_timestamp_ms == 123
            assert len(commands) > before_commands
            assert all(all(value == 0 for value in frame) for frame in commands)

            invalid = encode_remote(789, switches=(0, 1))
            spin_until(
                lambda: messages[-1].mcu_timestamp_ms == 789,
                packet=telemetry + invalid,
            )
            assert messages[-1].frame_fresh
            assert not messages[-1].controls_valid
            # MCU 重启后时间戳回退也能恢复接收，不按 MCU 时钟做 USB 超时判断。
            spin_until(
                lambda: messages[-1].mcu_timestamp_ms == 1,
                packet=telemetry + encode_remote(1),
            )
            assert messages[-1].frame_fresh and messages[-1].controls_valid

            # 物理串口断开后必须清空状态，不能保留看似可用的遥控输入。
            os.close(master)
            master = -1
            spin_until(lambda: messages[-1].mcu_timestamp_ms == 0)
            assert not messages[-1].frame_fresh
            assert not messages[-1].controls_valid
            assert list(messages[-1].channels) == [0, 0, 0, 0]
            assert (messages[-1].sw_right, messages[-1].sw_left) == (0, 0)
            assert messages[-1].header.stamp.sec == 0
            assert messages[-1].header.stamp.nanosec == 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process is not None:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        if master >= 0:
            os.close(master)
