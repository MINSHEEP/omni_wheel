"""建图首帧门控状态机单元测试。"""

import math

from kiwi_slam.mapping_gate import MappingGateConfig, MappingStartupGate


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
