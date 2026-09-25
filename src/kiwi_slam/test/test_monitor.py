"""Unit tests for hardware-independent health calculations."""

import math
from pathlib import Path

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

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
