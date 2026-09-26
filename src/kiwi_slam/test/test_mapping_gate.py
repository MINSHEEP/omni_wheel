"""建图首帧门控状态机单元测试。"""

import math
from pathlib import Path

import pytest
import yaml

from kiwi_slam.mapping_gate import MappingGateConfig, MappingStartupGate


@pytest.mark.parametrize('filename,tracked', [('tracked_mapping_gate.yaml', True),
                                           ('mapping_gate.yaml', False)])
def test_ready_qos_preserves_omni_latched_calibration_and_tracked_freshness(monkeypatch, filename, tracked):
    from unittest.mock import Mock
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
    from kiwi_slam.mapping_gate import MappingScanGateNode

    config = yaml.safe_load((Path(__file__).parents[1]/'config'/filename).read_text())
    parameters = config['kiwi_mapping_scan_gate']['ros__parameters']
    readers, writers = {}, {}
    monkeypatch.setattr(Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, name, default:
                        Parameter(name, value=parameters.get(name, default)))
    def subscription(self, message_type, topic, callback, qos):
        readers[topic] = qos
        return Mock()
    def publisher(self, message_type, topic, qos):
        writers[topic] = qos
        return Mock()
    monkeypatch.setattr(Node, 'create_subscription', subscription)
    monkeypatch.setattr(Node, 'create_publisher', publisher)
    monkeypatch.setattr(Node, 'create_timer', Mock())
    monkeypatch.setattr(Node, 'get_logger', Mock())
    MappingScanGateNode()
    ready = readers['/chassis/imu/calibration_ready']
    if tracked:
        for topic in ('/chassis/imu/calibration_ready', '/chassis/odom/laser_ready',
                      '/chassis/estimation/valid', '/chassis/estimation/state'):
            assert readers[topic].depth == 10
            assert readers[topic].durability == DurabilityPolicy.VOLATILE
            assert readers[topic].reliability == ReliabilityPolicy.RELIABLE
    else:
        # kiwi_base校准成功只发一次，建图后启动必须能读取已经提交的状态。
        assert ready.durability == DurabilityPolicy.TRANSIENT_LOCAL
        assert ready.depth == 1
    assert writers['/kiwi_slam/mapping_gate_ready'].durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert writers['/kiwi_slam/mapping_gate_ready'].depth == 1


def make_gate(required_scans=3):
    config = MappingGateConfig(
        stationary_duration_s=0.5,
        max_linear_speed_m_s=0.02,
        max_angular_speed_rad_s=0.03,
        max_yaw_step_rad=0.01,
        max_message_age_s=0.20,
        required_stable_scans=required_scans,
        min_valid_scan_ratio=0.25,
        min_scan_overlap_ratio=0.50,
        max_median_range_delta_m=0.05,
    )
    return MappingStartupGate(config)


def observe_stationary(gate, start=0.0, yaw=0.0):
    for index in range(7):
        gate.observe_odometry(start + index * 0.1, 0.0, 0.0, 0.0, yaw)


def test_gate_never_opens_before_real_imu_calibration_ready():
    gate = make_gate()
    observe_stationary(gate)
    for index in range(5):
        assert not gate.observe_scan(0.61 + index * 0.05, [1.0] * 8, 0.1, 5.0)
    assert not gate.opened


def test_consistent_scans_open_gate_and_first_released_scan_is_fresh():
    gate = make_gate(required_scans=3)
    gate.set_imu_ready(True)
    observe_stationary(gate)
    assert not gate.observe_scan(
        0.61, [1.00, 1.01, 2.0, math.inf] * 2, 0.1, 5.0
    )
    assert not gate.observe_scan(
        0.66, [1.01, 1.00, 2.01, math.inf] * 2, 0.1, 5.0
    )
    assert gate.observe_scan(0.71, [1.00, 1.02, 2.00, math.inf] * 2, 0.1, 5.0)
    assert gate.opened
    # 打开后不再因正常建图运动关闭，避免 /scan_slam 在运动中反复断流。
    gate.observe_odometry(0.72, 0.3, 0.0, 0.4, 1.0)
    assert gate.observe_scan(0.73, [math.inf] * 8, 0.1, 5.0)


def test_motion_and_message_gap_clear_all_startup_evidence():
    gate = make_gate(required_scans=2)
    gate.set_imu_ready(True)
    observe_stationary(gate)
    assert not gate.observe_scan(0.61, [1.0] * 8, 0.1, 5.0)

    gate.observe_odometry(0.65, 0.03, 0.0, 0.0, 0.0)
    assert gate.stable_scan_count == 0
    observe_stationary(gate, start=1.0)
    assert not gate.observe_scan(1.61, [1.0] * 8, 0.1, 5.0)

    # 0.5 s 断流超过 0.20 s 合约，不能与断流前静止区间拼接。
    gate.observe_odometry(2.1, 0.0, 0.0, 0.0, 0.0)
    assert gate.stationary_since == 2.1
    assert gate.stable_scan_count == 0


def test_yaw_wrap_is_small_step_but_real_rotation_resets_gate():
    gate = make_gate()
    gate.set_imu_ready(True)
    gate.observe_odometry(0.0, 0.0, 0.0, 0.0, math.pi - 0.004)
    gate.observe_odometry(0.1, 0.0, 0.0, 0.0, -math.pi + 0.004)
    assert gate.stationary_since == 0.0

    gate.observe_odometry(0.2, 0.0, 0.0, 0.0, -math.pi + 0.03)
    assert gate.stationary_since is None
    gate.observe_odometry(0.3, 0.0, 0.0, 0.0, -math.pi + 0.03)
    assert gate.stationary_since == 0.3


def test_sparse_or_inconsistent_scan_restarts_consecutive_count():
    gate = make_gate(required_scans=3)
    gate.set_imu_ready(True)
    observe_stationary(gate)
    good = [1.0, 1.0, 2.0, 2.0, math.inf, math.inf, math.inf, math.inf]
    assert not gate.observe_scan(0.61, good, 0.1, 5.0)
    assert gate.stable_scan_count == 1

    sparse = [
        1.0,
        math.inf,
        math.inf,
        math.inf,
        math.inf,
        math.inf,
        math.inf,
        math.inf,
    ]
    assert not gate.observe_scan(0.65, sparse, 0.1, 5.0)
    assert gate.stable_scan_count == 0

    assert not gate.observe_scan(0.66, good, 0.1, 5.0)
    shifted = [1.5, 1.5, 2.5, 2.5, math.inf, math.inf, math.inf, math.inf]
    assert not gate.observe_scan(0.67, shifted, 0.1, 5.0)
    assert gate.stable_scan_count == 1


def test_stale_odom_and_non_finite_values_are_rejected():
    gate = make_gate(required_scans=2)
    gate.set_imu_ready(True)
    observe_stationary(gate)
    assert not gate.observe_scan(1.0, [1.0] * 8, 0.1, 5.0)
    gate.observe_odometry(1.1, math.nan, 0.0, 0.0, 0.0)
    assert gate.stationary_since is None


def test_invalid_configuration_is_rejected():
    try:
        MappingGateConfig(0.5, 0.02, 0.03, 0.01, 0.2, 1, 0.1, 0.5, 0.05)
    except ValueError as error:
        assert "required_stable_scans" in str(error)
    else:
        raise AssertionError("invalid mapping gate configuration was accepted")


def test_tracked_gate_requires_fresh_imu_and_laser_odometry():
    config = MappingGateConfig(
        stationary_duration_s=0.5,
        max_linear_speed_m_s=0.02,
        max_angular_speed_rad_s=0.03,
        max_yaw_step_rad=0.01,
        max_message_age_s=0.20,
        required_stable_scans=2,
        min_valid_scan_ratio=0.25,
        min_scan_overlap_ratio=0.50,
        max_median_range_delta_m=0.05,
        require_imu_samples=True,
        require_source_odom=True,
    )
    gate = MappingStartupGate(config)
    gate.set_imu_ready(True)  # 履带无原 Kiwi 电机标定 Bool。
    observe_stationary(gate)
    assert not gate.observe_scan(0.61, [1.0] * 8, 0.1, 5.0)
    gate.observe_imu(0.60, True)
    assert not gate.observe_scan(0.61, [1.0] * 8, 0.1, 5.0)
    gate.observe_source_odom(0.60, True)
    assert not gate.observe_scan(0.61, [1.0] * 8, 0.1, 5.0)
    assert gate.observe_scan(0.66, [1.0] * 8, 0.1, 5.0)
    # 已开门后任一传感器断流，都暂停 /scan_slam，恢复后再转发。
    assert not gate.observe_scan(0.90, [1.0] * 8, 0.1, 5.0)
    gate.observe_imu(0.90, True)
    assert not gate.observe_scan(0.91, [1.0] * 8, 0.1, 5.0)
    gate.observe_source_odom(0.91, True)
    gate.observe_odometry(0.91, 0.0, 0.0, 0.0, 0.0)
    assert gate.observe_scan(0.92, [1.0] * 8, 0.1, 5.0)
    gate.observe_imu(0.93, False)
    assert not gate.observe_scan(0.94, [1.0] * 8, 0.1, 5.0)


def test_tracked_gate_pauses_when_filtered_odometry_stops():
    config = MappingGateConfig(
        0.5, 0.02, 0.03, 0.01, 0.20, 2, 0.25, 0.50, 0.05,
        require_imu_samples=True, require_source_odom=True,
    )
    gate = MappingStartupGate(config)
    gate.set_imu_ready(True)
    observe_stationary(gate)
    gate.observe_imu(0.60, True)
    gate.observe_source_odom(0.60, True)
    assert not gate.observe_scan(0.61, [1.0] * 8, 0.1, 5.0)
    assert gate.observe_scan(0.66, [1.0] * 8, 0.1, 5.0)
    gate.observe_imu(0.85, True)
    gate.observe_source_odom(0.85, True)
    assert not gate.observe_scan(0.85, [1.0] * 8, 0.1, 5.0)
    gate.observe_odometry(0.86, 0.2, 0.0, 0.1, 0.1)
    assert gate.observe_scan(0.87, [1.0] * 8, 0.1, 5.0)


def test_calibration_revocation_closes_an_already_open_gate():
    gate = make_gate(required_scans=2)
    gate.set_imu_ready(True)
    observe_stationary(gate)
    gate.observe_scan(.61, [1.]*8, .1, 5.)
    assert gate.observe_scan(.66, [1.]*8, .1, 5.)
    gate.set_imu_ready(False)
    assert not gate.opened
    assert not gate.observe_scan(.67, [1.]*8, .1, 5.)


def test_estimation_invalid_closes_gate_even_with_recent_odometry(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from kiwi_slam.mapping_gate import MappingScanGateNode
    monkeypatch.setattr('kiwi_slam.mapping_gate.time.monotonic',lambda:10.)
    node=SimpleNamespace(quality_ready=True, quality_time=9.9, imu_ready_time=9.9,
        gate=SimpleNamespace(imu_ready=True, opened=True, _sources_ready=lambda t:True),
        require_estimation_ready=True, estimation_ready=False, estimation_time=9.9,
        estimation_state="VALID", estimation_state_time=9.9,
        ready_pub=Mock())
    assert not MappingScanGateNode._health(node)
    assert node.ready_pub.publish.call_args.args[0].data is False
    node.estimation_ready=True
    assert MappingScanGateNode._health(node)
    node.estimation_time=9.
    assert not MappingScanGateNode._health(node)


def test_inertial_tf_does_not_authorize_map_writes_before_filter_recovery(monkeypatch):
    from types import SimpleNamespace
    from unittest.mock import Mock
    from kiwi_slam.mapping_gate import MappingScanGateNode
    monkeypatch.setattr('kiwi_slam.mapping_gate.time.monotonic', lambda: 10.)
    node = SimpleNamespace(quality_ready=True, quality_time=9.9, imu_ready_time=9.9,
        gate=SimpleNamespace(imu_ready=True, opened=True, _sources_ready=lambda t: True),
        require_estimation_ready=True, estimation_ready=True, estimation_time=9.9,
        estimation_state='INERTIAL', estimation_state_time=9.9, ready_pub=Mock())
    assert not MappingScanGateNode._health(node)
    node.estimation_state = 'VALID'
    assert MappingScanGateNode._health(node)
    node.estimation_state_time = 9.0
    assert not MappingScanGateNode._health(node)
