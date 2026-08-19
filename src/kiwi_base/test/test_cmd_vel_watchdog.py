"""通过伪终端验证真实底盘节点的 cmd_vel 超时停车链路。"""

import errno
import math
import os
import pty
import struct
import subprocess
import time
from pathlib import Path

import rclpy
import pytest
from geometry_msgs.msg import Twist
from kiwi_base.msg import FeedbackDebug
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState


PACKAGE = Path(__file__).resolve().parents[1]
FRAME_HEADER = bytes((0x33, 12, 0x3A))
FRAME_SIZE = 18
FEEDBACK_PAYLOAD_SIZE = 48


def find_node_executable():
    """从已 source 的安装前缀中定位待测节点。"""
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        candidate = Path(prefix) / "lib" / "kiwi_base" / "kiwi_base_node"
        if candidate.is_file():
            return candidate
    raise AssertionError("未在 AMENT_PREFIX_PATH 中找到 kiwi_base_node")


def read_command_frames(master_fd, buffer):
    """非阻塞读取并从串口字节流提取固定长度 0x3A 帧。"""
    while True:
        try:
            chunk = os.read(master_fd, 4096)
        except BlockingIOError:
            break
        except OSError:
            # 节点尚未打开伪终端从端时，主端可能暂时返回 EIO。
            break
        if not chunk:
            break
        buffer += chunk

    frames = []
    while True:
        offset = buffer.find(FRAME_HEADER)
        if offset < 0:
            buffer = buffer[-2:]
            break
        if len(buffer) < offset + FRAME_SIZE:
            buffer = buffer[offset:]
            break
        frame = buffer[offset:offset + FRAME_SIZE]
        frames.append(struct.unpack("<fff", frame[4:16]))
        buffer = buffer[offset + FRAME_SIZE:]
    return frames, buffer


def crc8_dji(data):
    """计算协议头使用的 DJI 反射 CRC8。"""
    crc = 0xFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8C) if crc & 1 else crc >> 1
    return crc


def crc16_ccitt(data):
    """计算线上低字节在前的 DJI 反射 CRC16。"""
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if crc & 1 else crc >> 1
    return crc


def encode_feedback(values):
    """编码一帧合法 0x3B：motor/euler/gyro/accel 各三个 float32。"""
    assert len(values) == 12
    header = bytearray((0x33, FEEDBACK_PAYLOAD_SIZE, 0x3B))
    header.append(crc8_dji(header))
    frame = header + struct.pack("<12f", *values)
    frame += struct.pack("<H", crc16_ccitt(frame))
    return bytes(frame)


def quaternion_yaw(orientation):
    """从四元数提取 ROS yaw。"""
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


def test_no_ack_watchdog_and_feedback_odometry(monkeypatch, tmp_path):
    """无反馈时控制不阻塞；只有异步电机实测值能够驱动里程计。"""
    domain_id = str(100 + os.getpid() % 100)
    monkeypatch.setenv("ROS_DOMAIN_ID", domain_id)
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

    command = [
        str(find_node_executable()),
        "--ros-args",
        "--params-file",
        str(PACKAGE / "config" / "kiwi_base.yaml"),
        "-p",
        f"device:={device}",
        "-p",
        "imu_stationary_hold_s:=0.01",
        "-p",
        "imu_covariance_calibration_samples:=3",
        "-p",
        "imu_covariance_update_alpha:=1.0",
        "-p",
        "imu_yaw_drift_calibration_samples:=0",
        "-p",
        "imu_gyro_calibration_samples:=0",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
    )

    publisher_node = None
    try:
        rclpy.init()
        publisher_node = rclpy.create_node("kiwi_watchdog_test_publisher")
        publisher = publisher_node.create_publisher(Twist, "/cmd_vel", 10)
        odom_twists = []
        odom_positions = []
        joint_velocities = []
        imu_messages = []
        feedback_messages = []

        def on_odometry(message):
            """同时保存速度与积分位姿，验证反馈里程计的完整输出。"""
            odom_twists.append(message.twist.twist)
            odom_positions.append(
                (message.pose.pose.position.x, message.pose.pose.position.y)
            )

        publisher_node.create_subscription(
            Odometry,
            "/chassis/odom/raw",
            on_odometry,
            10,
        )
        publisher_node.create_subscription(
            JointState,
            "/joint_states",
            lambda message: joint_velocities.append(tuple(message.velocity)),
            10,
        )
        publisher_node.create_subscription(
            Imu,
            "/chassis/imu/data_raw",
            imu_messages.append,
            10,
        )
        publisher_node.create_subscription(
            FeedbackDebug,
            "/chassis/feedback/raw",
            feedback_messages.append,
            10,
        )
        serial_buffer = b""

        discovery_deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() == 0:
            assert process.poll() is None, "kiwi_base_node 在 DDS 发现前退出"
            assert time.monotonic() < discovery_deadline, "等待 /cmd_vel 订阅者超时"
            rclpy.spin_once(publisher_node, timeout_sec=0.05)

        # 节点打开设备后必须设置 TIOCEXCL。该检查覆盖“另一个 ROS_DOMAIN_ID
        # 又启动一套底盘节点”的现场故障：第二次 open 应得到 EBUSY，不能与首个
        # 进程随机分流同一条 0x3B 字节流。
        with pytest.raises(OSError) as exclusive_error:
            os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        assert exclusive_error.value.errno == errno.EBUSY

        # 清除节点启动后、收到命令前持续发送的零速帧。
        time.sleep(0.1)
        _, serial_buffer = read_command_frames(master_fd, serial_buffer)

        message = Twist()
        message.linear.x = 0.4
        message.linear.y = 0.2
        message.angular.z = 0.3

        nonzero_seen = False
        publish_deadline = time.monotonic() + 2.0
        while not nonzero_seen:
            publisher.publish(message)
            rclpy.spin_once(publisher_node, timeout_sec=0.02)
            frames, serial_buffer = read_command_frames(master_fd, serial_buffer)
            nonzero_seen = any(any(abs(value) > 1e-3 for value in frame) for frame in frames)
            assert process.poll() is None, "kiwi_base_node 在接收非零命令后退出"
            assert time.monotonic() < publish_deadline, (
                "未观测到非零串口指令"
            )

        # 0x3B 尚未注入时绝不能用 /cmd_vel 伪造里程计、关节或 IMU 遥测。
        assert not odom_twists
        assert not joint_velocities
        assert not imu_messages
        assert not feedback_messages

        # 最后一条消息之后不再发布；从此刻测量首个零速帧到达时间。
        publisher.publish(message)
        interruption_time = time.monotonic()
        zero_frame_time = None
        stop_deadline = interruption_time + 1.0
        while zero_frame_time is None:
            rclpy.spin_once(publisher_node, timeout_sec=0.01)
            frames, serial_buffer = read_command_frames(master_fd, serial_buffer)
            if any(all(abs(value) < 1e-6 for value in frame) for frame in frames):
                zero_frame_time = time.monotonic()
                break
            assert process.poll() is None, "kiwi_base_node 在看门狗触发前退出"
            assert time.monotonic() < stop_deadline, "看门狗未在 1 秒内发送零速帧"

        elapsed = zero_frame_time - interruption_time
        # 50 Hz 循环有一个周期量化裕度，DDS 传输和测试调度再保留 0.10 秒。
        assert 0.40 <= elapsed <= 0.62, f"看门狗归零耗时异常: {elapsed:.3f} s"

        def inject_feedback(values):
            """注入一帧并等待 IMU/Debug 同步到达，同时持续排空 0x3A TX。"""
            nonlocal serial_buffer
            previous_imu_count = len(imu_messages)
            previous_debug_count = len(feedback_messages)
            frame = encode_feedback(values)
            assert os.write(master_fd, frame) == len(frame)
            deadline = time.monotonic() + 1.0
            while (
                len(imu_messages) == previous_imu_count
                or len(feedback_messages) == previous_debug_count
            ):
                rclpy.spin_once(publisher_node, timeout_sec=0.02)
                _, serial_buffer = read_command_frames(master_fd, serial_buffer)
                assert process.poll() is None, "注入 0x3B 后 kiwi_base_node 意外退出"
                assert time.monotonic() < deadline, "等待异步 IMU/Debug 遥测超时"
            return imu_messages[-1], feedback_messages[-1]

        # 首帧建立时间参考，第二帧开始静止候选，随后三帧完成一个非重叠窗口。
        # yaw 以低于静止门限的线性漂移跨越 ±pi，可同时验证展开、去趋势方差与
        # 惯性静止门不会把正常传感器细微漂移误判为运动。
        stationary_frames = [
            (3.1410, -0.10),
            (3.1412, -0.11),
            (3.1414, -0.10),
            (-3.141585, -0.12),
            (-3.141385, -0.08),
        ]
        latest_imu = None
        latest_debug = None
        for yaw_raw, gyro_z_raw in stationary_frames:
            latest_imu, latest_debug = inject_feedback(
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    yaw_raw,
                    0.0,
                    0.0,
                    gyro_z_raw,
                    -7.87,
                    0.05,
                    -5.86,
                ]
            )
            time.sleep(0.03)

        assert latest_imu is not None and latest_debug is not None
        measured_yaw = quaternion_yaw(latest_imu.orientation)
        assert math.remainder(measured_yaw - stationary_frames[-1][0], 2.0 * math.pi) == (
            pytest.approx(0.0, abs=1e-5)
        )
        assert latest_debug.yaw_rad == pytest.approx(stationary_frames[-1][0], abs=1e-6)
        yaw_covariance = latest_imu.orientation_covariance[8]
        gyro_covariance = latest_imu.angular_velocity_covariance[8]
        assert math.isfinite(yaw_covariance)
        assert math.isfinite(gyro_covariance)
        assert 1.0e-4 <= yaw_covariance < 0.001
        assert gyro_covariance > 1.0e-4

        # 连续注入纯 +X 电机反馈，使一阶低通收敛；运动遥测会立即终止当前
        # 静止协方差窗口，但必须驱动实际反馈 odom/joint_states。
        odom_twists.clear()
        odom_positions.clear()
        joint_velocities.clear()
        moving_imu = None
        moving_debug = None
        for _ in range(8):
            moving_imu, moving_debug = inject_feedback(
                [
                    19.0,
                    0.0,
                    -19.0,
                    0.0,
                    0.0,
                    0.25,
                    0.0,
                    0.0,
                    -0.20,
                    -7.87,
                    0.05,
                    -5.86,
                ]
            )
            time.sleep(0.03)
        assert moving_imu is not None and moving_debug is not None
        assert moving_debug.motor_1_actual_rad_s == pytest.approx(19.0)
        # 运动只清空统计窗口，必须保留上一个已提交值，禁止 EKF 权重跳回启动值。
        assert moving_imu.orientation_covariance[8] == pytest.approx(yaw_covariance)
        assert moving_imu.angular_velocity_covariance[8] == pytest.approx(
            gyro_covariance
        )
        feedback_deadline = time.monotonic() + 0.5
        while not odom_twists or not joint_velocities:
            rclpy.spin_once(publisher_node, timeout_sec=0.02)
            assert time.monotonic() < feedback_deadline, "等待反馈里程计消息超时"
        # [19, 0, -19] / 19 = [1, 0, -1] rad/s，对应纯 +X 约 0.1172 m/s。
        assert odom_twists[-1].linear.x == pytest.approx(
            0.1015 / math.sin(math.pi / 3.0), rel=0.02
        )
        assert odom_twists[-1].linear.y == pytest.approx(0.0, abs=1e-5)
        assert odom_twists[-1].angular.z == pytest.approx(0.0, abs=1e-5)
        assert joint_velocities[-1] == pytest.approx((1.0, 0.0, -1.0), rel=0.02)
        assert odom_positions[-1][0] > 0.0
        assert odom_positions[-1][1] == pytest.approx(0.0, abs=1e-5)

        # 断开伪终端后继续发送非零目标；节点不得退出，也不得把仅存在于主机
        # 内存中的 /cmd_vel 积分成新的里程计消息。
        for _ in range(5):
            rclpy.spin_once(publisher_node, timeout_sec=0.02)
        odom_count_before_disconnect = len(odom_twists)
        os.close(master_fd)
        master_fd = -1
        disconnect_deadline = time.monotonic() + 1.0
        while time.monotonic() < disconnect_deadline:
            publisher.publish(message)
            rclpy.spin_once(publisher_node, timeout_sec=0.03)
            assert process.poll() is None, "串口断开后 kiwi_base_node 意外退出"
        assert len(odom_twists) == odom_count_before_disconnect
    finally:
        if publisher_node is not None:
            publisher_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3.0)
        if master_fd >= 0:
            os.close(master_fd)
