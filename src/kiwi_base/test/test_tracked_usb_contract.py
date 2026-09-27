"""伪终端核对双履带 0x3A 指令映射、横移拒绝和断发停车。"""

import os
import pty
import struct
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist


PACKAGE = Path(__file__).resolve().parents[1]
HEADER = bytes((0x33, 12, 0x3A))


def find_executable():
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        candidate = Path(prefix) / "lib/kiwi_base/tracked_base_node"
        if candidate.is_file():
            return candidate
    raise AssertionError("tracked_base_node 未安装")


def receive_frames(master, buffer):
    while True:
        try:
            buffer += os.read(master, 4096)
        except (BlockingIOError, OSError):
            break
    frames = []
    while True:
        offset = buffer.find(HEADER)
        if offset < 0:
            return frames, buffer[-2:]
        if len(buffer) < offset + 18:
            return frames, buffer[offset:]
        frames.append(struct.unpack("<fff", buffer[offset + 4:offset + 16]))
        buffer = buffer[offset + 18:]


def test_calibrated_motion_and_watchdog(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    monkeypatch.setenv(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        str(PACKAGE / "test/fastdds_shm_profile.xml"),
    )
    master, slave = pty.openpty()
    device = os.ttyname(slave)
    os.set_blocking(master, False)
    os.close(slave)
    process = subprocess.Popen(
        [str(find_executable()), "--ros-args", "--params-file",
         str(PACKAGE / "config/tracked_laser.yaml"), "-p", f"device:={device}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("tracked_usb_contract_test")
        pub = node.create_publisher(Twist, "/cmd_vel", 10)
        deadline = time.monotonic() + 5.0
        while pub.get_subscription_count() == 0:
            assert process.poll() is None, "履带节点提前退出"
            assert time.monotonic() < deadline, "等待 /cmd_vel 订阅者超时"
            rclpy.spin_once(node, timeout_sec=0.05)

        buffer = b""
        command = Twist()
        command.linear.x = 0.2
        command.angular.z = 0.4
        deadline = time.monotonic() + 2.0
        while True:
            pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = receive_frames(master, buffer)
            if any(abs(left - 2.12) < 0.001 and abs(right - 6.36) < 0.001
                   and third == 0.0 for left, right, third in frames):
                break
            assert process.poll() is None
            assert time.monotonic() < deadline, "未收到正确的左右履带指令"

        # 超过 200 ms 无新 /cmd_vel，电机指令必须恢复零。
        deadline = time.monotonic() + 0.7
        while True:
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = receive_frames(master, buffer)
            if (0.0, 0.0, 0.0) in frames:
                break
            assert time.monotonic() < deadline, "断发后未停车"

        command.linear.y = 0.1
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            pub.publish(command)
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = receive_frames(master, buffer)
            assert all(frame == (0.0, 0.0, 0.0) for frame in frames)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        process.terminate()
        try:
            process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.communicate(timeout=3.0)
        os.close(master)
