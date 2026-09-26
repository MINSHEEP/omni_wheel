"""Unit tests for hardware-independent health calculations."""

import math
import time
from pathlib import Path

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String

from kiwi_slam.health import quaternion_norm, sample_rate_hz, valid_range_ratio
from kiwi_slam.monitor import KiwiSlamMonitor


def test_valid_range_ratio_rejects_nan_inf_and_out_of_range():
    ranges = [0.05, 1.0, 12.0, math.inf, math.nan, 0.01, 13.0]
    assert valid_range_ratio(ranges, 0.05, 12.0) == 3 / 7


def test_valid_range_ratio_rejects_invalid_configuration():
    assert valid_range_ratio([], 0.05, 12.0) == 0.0
    assert valid_range_ratio([1.0], 12.0, 0.05) == 0.0


def test_sample_rate_and_quaternion_norm():
    assert sample_rate_hz([1.0, 1.1, 1.2]) == pytest.approx(10.0)
    assert sample_rate_hz([1.0]) == 0.0
    assert quaternion_norm(0.0, 0.0, 0.0, 1.0) == 1.0


def test_monitor_can_change_log_severity_without_crashing(tmp_path, monkeypatch):
    """复现远程日志中的 ERROR→INFO 切换，确保 Humble logger 不再抛 ValueError。"""
    config = Path(__file__).resolve().parents[1] / "config" / "monitor.yaml"
    # 测试可能运行在主目录只读的 CI/沙箱中，将 ROS 日志显式写入 pytest 临时目录。
    # 保留日志初始化可以覆盖真实节点的 logger 路径，同时避免依赖开发机的 ~/.ros 权限。
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    node = KiwiSlamMonitor()
    try:
        node._log_report(DiagnosticStatus.ERROR, "error state")
        node._log_report(DiagnosticStatus.OK, "normal state")
        node._log_report(DiagnosticStatus.WARN, "warning state")
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.width = grid.info.height = 1
        grid.data = [0]
        node.last_map = grid
        node.map_received = 0.0
        assert node._stream_status()[-1].message == "map update timeout"
        node.map_is_static = True
        assert node._stream_status()[-1].level == DiagnosticStatus.OK
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_monitor_reports_stale_map_tf_even_when_transform_is_available(tmp_path, monkeypatch):
    """重现扫描被丢弃后 map→odom 仍周期发布旧变换的现场故障。"""
    config = Path(__file__).resolve().parents[1] / "config" / "monitor.yaml"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    node = KiwiSlamMonitor()
    try:
        now_ns = node.get_clock().now().nanoseconds

        class RecordedTf:
            def lookup_transform(self, target, source, _time):
                transform = TransformStamped()
                transform.header.frame_id = target
                transform.child_frame_id = source
                if (target, source) == ("map", "odom"):
                    stamp_ns = now_ns - 135_000_000_000
                elif (target, source) == ("odom", "base_footprint"):
                    stamp_ns = now_ns - 100_000_000
                else:
                    stamp_ns = 0  # URDF 静态变换
                transform.header.stamp.sec = stamp_ns // 1_000_000_000
                transform.header.stamp.nanosec = stamp_ns % 1_000_000_000
                return transform

        node.tf_buffer = RecordedTf()
        statuses = {item.name: item for item in node._tf_status()}
        assert statuses["kiwi_slam/map_to_odom_tf"].level == DiagnosticStatus.ERROR
        assert statuses["kiwi_slam/map_to_odom_tf"].message == "transform stale"
        assert statuses["kiwi_slam/odom_to_base_tf"].level == DiagnosticStatus.OK
        assert statuses["kiwi_slam/base_to_laser_tf"].level == DiagnosticStatus.OK
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def progress_monitor(tmp_path, monkeypatch):
    config = Path(__file__).resolve().parents[1] / "config" / "monitor.yaml"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    rclpy.init(args=[
        "--ros-args", "--params-file", str(config),
        "-p", "mapping_gate_ready_topic:=/kiwi_slam/mapping_gate_ready",
        "-p", "estimation_state_topic:=/chassis/estimation/state",
    ])
    node = KiwiSlamMonitor()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def recorded_map(node, age_sec=120.0):
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    stamp = node.get_clock().now().nanoseconds - int(age_sec * 1e9)
    grid.header.stamp.sec, grid.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    grid.info.width = grid.info.height = 1
    grid.data = [0]
    node._on_map(grid)
    return grid


def test_republished_map_reports_paused_estimation(progress_monitor):
    node = progress_monitor
    grid = recorded_map(node)
    node.map_stamp_changed_at -= 60.0
    original_change = node.map_stamp_changed_at
    node._on_map(grid)
    node._on_estimation_state(String(data="PAUSED"))
    node._on_mapping_gate(Bool(data=False))
    status = node._stream_status()[-1]
    details = {item.key: item.value for item in status.values}
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "mapping paused: estimation PAUSED"
    assert node.map_stamp_changed_at == original_change
    assert float(details["age_sec"]) < 1.0
    assert float(details["header_age_sec"]) >= 119.0
    assert float(details["stamp_unchanged_sec"]) >= 60.0


def test_inertial_position_continuity_does_not_claim_map_updates(progress_monitor):
    node = progress_monitor
    recorded_map(node)
    node._on_estimation_state(String(data="INERTIAL"))
    node._on_mapping_gate(Bool(data=False))
    status = node._stream_status()[-1]
    assert status.level == DiagnosticStatus.WARN
    assert status.message == "mapping paused: inertial prediction only"


def test_stationary_old_keyframe_is_valid_with_healthy_sources(progress_monitor):
    node = progress_monitor
    recorded_map(node)
    node.map_stamp_changed_at -= 60.0
    node._on_estimation_state(String(data="VALID"))
    node._on_mapping_gate(Bool(data=True))
    odom = Odometry()
    odom.header.frame_id = node.odom_frame
    odom.child_frame_id = node.base_frame
    odom.header.stamp = node.get_clock().now().to_msg()
    odom.pose.pose.orientation.w = 1.0
    node._on_odom(odom)
    assert node._stream_status()[-1].level == DiagnosticStatus.OK


def test_static_map_does_not_require_mapping_state_heartbeats(progress_monitor):
    node = progress_monitor
    recorded_map(node)
    node.map_is_static = True
    node.map_received = 0.0
    node._on_estimation_state(String(data="PAUSED"))
    node._on_mapping_gate(Bool(data=False))
    assert node._stream_status()[-1].level == DiagnosticStatus.OK


@pytest.mark.parametrize("source, message", [
    ("estimation", "estimation state timeout"),
    ("gate", "mapping gate state timeout"),
])
def test_stale_gate_heartbeats_cannot_claim_a_current_map(progress_monitor, source, message):
    node = progress_monitor
    recorded_map(node)
    node._on_estimation_state(String(data="VALID"))
    node._on_mapping_gate(Bool(data=True))
    if source == "estimation":
        node.estimation_state_received -= node.timeout_sec + 1.0
    else:
        node.mapping_gate_received -= node.timeout_sec + 1.0
    status = node._stream_status()[-1]
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == message


def test_gate_can_pause_mapping_while_odometry_is_valid(progress_monitor):
    node = progress_monitor
    recorded_map(node)
    node._on_estimation_state(String(data="VALID"))
    node._on_mapping_gate(Bool(data=False))
    status = node._stream_status()[-1]
    assert status.level == DiagnosticStatus.WARN
    assert status.message == "mapping paused: gate not ready"


def test_default_monitor_catches_repeated_map_when_odometry_stops(progress_monitor):
    node = progress_monitor
    node.mapping_gate_ready_topic = node.estimation_state_topic = ""
    grid = recorded_map(node)
    node.map_stamp_changed_at -= 60.0
    node._on_map(grid)
    status = node._stream_status()[-1]
    assert status.level == DiagnosticStatus.ERROR
    assert status.message == "map stale: upstream odometry unavailable"
    # 真正的新时间戳（包括 /clock 重置后的时间）应重置计时，不沿用旧停更判断。
    grid.header.stamp.sec += 1
    node._on_map(grid)
    assert time.monotonic() - node.map_stamp_changed_at < 1.0
    assert node._stream_status()[-1].level == DiagnosticStatus.OK


def test_inertial_unavailable_is_visible_even_when_mapping_works(tmp_path, monkeypatch):
    config = Path(__file__).resolve().parents[1] / "config" / "monitor.yaml"
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "ros_log"))
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    node = KiwiSlamMonitor()
    try:
        assert node._inertial_status() == []  # 全向不要求履带INS话题。
        node.inertial_valid_topic = "/chassis/inertial/valid"
        node._on_inertial_valid(Bool(data=False))
        node._on_inertial_reason(String(data="position_uncertainty_limit"))
        status = node._inertial_status()[0]
        assert status.level == DiagnosticStatus.WARN
        assert "position_uncertainty_limit" in str(status.values)
        node._on_inertial_valid(Bool(data=True))
        assert node._inertial_status()[0].level == DiagnosticStatus.OK
        node.inertial_received -= node.timeout_sec+1.
        assert node._inertial_status()[0].level == DiagnosticStatus.WARN
    finally:
        node.destroy_node()
        rclpy.shutdown()
