"""通过伪终端验证履带原始试车命令、限幅和断发停车。"""

import os
import pty
import struct
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


PACKAGE = Path(__file__).resolve().parents[1]
FRAME_HEADER = bytes((0x33, 12, 0x3A))


def find_node_executable():
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        candidate = Path(prefix) / "lib" / "kiwi_base" / "tracked_base_node"
        if candidate.is_file():
            return candidate
    raise AssertionError("未找到 tracked_base_node")


def read_frames(master_fd, buffer):
    while True:
        try:
            buffer += os.read(master_fd, 4096)
        except (BlockingIOError, OSError):
            break
    frames = []
    while True:
        offset = buffer.find(FRAME_HEADER)
        if offset < 0:
            return frames, buffer[-2:]
        if len(buffer) < offset + 18:
            return frames, buffer[offset:]
        frames.append(struct.unpack("<fff", buffer[offset + 4:offset + 16]))
        buffer = buffer[offset + 18:]


def contains_motor_command(frames, left, right, tolerance=0.001):
    return any(
        abs(frame[0] - left) < tolerance and
        abs(frame[1] - right) < tolerance and
        frame[2] == 0.0
        for frame in frames
    )


def test_raw_motor_commands_are_separate_from_cmd_vel(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    monkeypatch.setenv(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        str(PACKAGE / "test" / "fastdds_shm_profile.xml"),
    )
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.set_blocking(master_fd, False)
    os.close(slave_fd)
    process = subprocess.Popen(
        [
            str(find_node_executable()),
            "--ros-args", "--params-file", str(PACKAGE / "config" / "tracked_pwm.yaml"),
            "-p", f"device:={device}",
            "-p", "hardware_calibrated:=false",
            "-p", "enable_motion:=false",
            "-p", "keyboard_trial_enabled:=false",
            "-p", "open_loop_test_enabled:=true",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("tracked_open_loop_test_publisher")
        raw_pub = node.create_publisher(
            Float32MultiArray, "/chassis/open_loop_motor_cmd", 10
        )
        cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        deadline = time.monotonic() + 5.0
        while raw_pub.get_subscription_count() == 0:
            assert process.poll() is None, "履带节点在 DDS 发现前退出"
            assert time.monotonic() < deadline, "等待原始命令订阅者超时"
            rclpy.spin_once(node, timeout_sec=0.05)

        buffer = b""
        cmd = Twist()
        cmd.linear.x = 0.2
        until = time.monotonic() + 0.35
        while time.monotonic() < until:
            cmd_pub.publish(cmd)
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = read_frames(master_fd, buffer)
            assert all(frame == (0.0, 0.0, 0.0) for frame in frames)

        test_cmd = Float32MultiArray(data=[5.0, -5.0])
        deadline = time.monotonic() + 2.0
        seen = False
        while not seen:
            raw_pub.publish(test_cmd)
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = read_frames(master_fd, buffer)
            seen = (5.0, -5.0, 0.0) in frames
            assert process.poll() is None
            assert time.monotonic() < deadline, "未收到原始左右履带命令"

        # 20 ms 发送周期、200 ms 超时；断发后必须恢复零命令。
        deadline = time.monotonic() + 0.6
        zero_seen = False
        while not zero_seen:
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = read_frames(master_fd, buffer)
            zero_seen = (0.0, 0.0, 0.0) in frames
            assert time.monotonic() < deadline, "断发后未停车"

        # 超限、长度错误和非有限值都不能变成电机运动命令。
        for values in ([11.0, 0.0], [1.0], [float("nan"), 0.0]):
            invalid = Float32MultiArray(data=values)
            until = time.monotonic() + 0.12
            while time.monotonic() < until:
                raw_pub.publish(invalid)
                rclpy.spin_once(node, timeout_sec=0.03)
                frames, buffer = read_frames(master_fd, buffer)
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
        os.close(master_fd)


def test_temporary_keyboard_trial_is_limited_and_stops_on_timeout(monkeypatch, tmp_path):
    monkeypatch.setenv("ROS_DOMAIN_ID", str(100 + os.getpid() % 100))
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    monkeypatch.setenv(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        str(PACKAGE / "test" / "fastdds_shm_profile.xml"),
    )
    master_fd, slave_fd = pty.openpty()
    device = os.ttyname(slave_fd)
    os.set_blocking(master_fd, False)
    os.close(slave_fd)
    process = subprocess.Popen(
        [
            str(find_node_executable()),
            "--ros-args", "--params-file", str(PACKAGE / "config" / "tracked_pwm.yaml"),
            "-p", f"device:={device}",
            "-p", "hardware_calibrated:=false",
            "-p", "enable_motion:=false",
            "-p", "keyboard_trial_enabled:=true",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )
    node = None
    try:
        rclpy.init()
        node = rclpy.create_node("tracked_keyboard_trial_publisher")
        cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        deadline = time.monotonic() + 5.0
        while cmd_pub.get_subscription_count() == 0:
            assert process.poll() is None, "履带节点在 DDS 发现前退出"
            assert time.monotonic() < deadline, "等待键盘话题订阅者超时"
            rclpy.spin_once(node, timeout_sec=0.05)

        buffer = b""
        forward = Twist()
        forward.linear.x = 0.235
        deadline = time.monotonic() + 2.0
        seen = False
        while not seen:
            cmd_pub.publish(forward)
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = read_frames(master_fd, buffer)
            assert all(max(abs(motor) for motor in frame[:2]) <= 5.0 for frame in frames)
            seen = contains_motor_command(frames, 4.982, 4.982)
            assert process.poll() is None
            assert time.monotonic() < deadline, "临时键盘前进命令未到达电机"

        backward = Twist()
        backward.linear.x = -0.235
        for command, expected, label in (
            (backward, (-4.982, -4.982), "后退"),
            (Twist(), (-5.0, 5.0), "左转"),
            (Twist(), (5.0, -5.0), "右转"),
        ):
            if label == "左转":
                command.angular.z = 0.95
            elif label == "右转":
                command.angular.z = -0.95
            deadline = time.monotonic() + 2.0
            seen = False
            while not seen:
                cmd_pub.publish(command)
                rclpy.spin_once(node, timeout_sec=0.03)
                frames, buffer = read_frames(master_fd, buffer)
                assert all(max(abs(motor) for motor in frame[:2]) <= 5.0 for frame in frames)
                seen = contains_motor_command(frames, *expected)
                assert process.poll() is None
                assert time.monotonic() < deadline, f"临时键盘{label}命令未到达电机"

        lateral = Twist()
        lateral.linear.y = 0.10
        cmd_pub.publish(lateral)
        deadline = time.monotonic() + 0.5
        stopped = False
        while not stopped:
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = read_frames(master_fd, buffer)
            stopped = (0.0, 0.0, 0.0) in frames
            assert time.monotonic() < deadline, "横移命令未停车"

        cmd_pub.publish(forward)
        deadline = time.monotonic() + 1.0
        while True:
            rclpy.spin_once(node, timeout_sec=0.03)
            frames, buffer = read_frames(master_fd, buffer)
            if contains_motor_command(frames, 4.982, 4.982):
                break
            assert time.monotonic() < deadline, "超时测试前未收到前进命令"
        time.sleep(0.3)
        deadline = time.monotonic() + 0.5
        while True:
            frames, buffer = read_frames(master_fd, buffer)
            if (0.0, 0.0, 0.0) in frames:
                break
            assert time.monotonic() < deadline, "停止键盘发布后未停车"
            time.sleep(0.02)
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
        os.close(master_fd)
