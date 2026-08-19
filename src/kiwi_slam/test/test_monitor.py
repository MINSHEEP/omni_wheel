"""Unit tests for hardware-independent health calculations."""

import math
from pathlib import Path

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus

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
    finally:
        node.destroy_node()
        rclpy.shutdown()
