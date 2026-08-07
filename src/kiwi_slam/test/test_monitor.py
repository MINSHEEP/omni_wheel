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


def test_monitor_can_change_log_severity_without_crashing():
    """复现远程日志中的 ERROR→INFO 切换，确保 Humble logger 不再抛 ValueError。"""
    config = Path(__file__).resolve().parents[1] / "config" / "monitor.yaml"
    rclpy.init(args=["--ros-args", "--params-file", str(config)])
    node = KiwiSlamMonitor()
    try:
        node._log_report(DiagnosticStatus.ERROR, "error state")
        node._log_report(DiagnosticStatus.OK, "normal state")
        node._log_report(DiagnosticStatus.WARN, "warning state")
    finally:
        node.destroy_node()
        rclpy.shutdown()
