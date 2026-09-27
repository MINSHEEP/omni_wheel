"""重力/前进轴独立激励、平面杆臂、成批时间戳与拒绝无效标定。"""
import numpy as np
import pytest

from kiwi_slam.planar_imu import AccelerationProcessor, PlanarCalibration, fit_calibration
from kiwi_slam.calibrate_planar_imu import static_samples


ROTATION = np.array([[-0.6, 0, 0.8], [0, 1, 0], [-0.8, 0, -0.6]])
REST = ROTATION.T @ [0, 0, 9.81]


def calibration():
    return PlanarCalibration(ROTATION, REST, [0.04] * 3)


def test_static_gravity_and_known_forward_acceleration_recover_body_axes():
    rng = np.random.default_rng(12)
    stationary = REST + rng.normal(0, 0.04, (2000, 3))
    reference = np.linspace(-0.4, 0.4, 100)
    movement = REST + np.outer(reference, ROTATION[0]) + rng.normal(0, 0.01, (100, 3))
    result, report = fit_calibration(stationary, movement, reference)
    assert report["forward_correlation"] > 0.99
    assert np.allclose(result.rotation, ROTATION, atol=0.02)
    assert np.allclose(result.correct(REST), 0, atol=0.003)
    assert np.allclose(result.correct(REST + ROTATION.T @ [0.2, -0.1, 0]),
                       [0.2, -0.1, 0], atol=0.006)


def test_rotation_at_offset_imu_does_not_invent_body_translation():
    wz, alpha = 1.0, 0.5
    body_at_sensor = np.array([-wz * wz * 0.08, alpha * 0.08, 0])
    corrected = calibration().correct(REST + ROTATION.T @ body_at_sensor, wz, alpha)
    assert np.allclose(corrected, 0, atol=1e-12)


def test_usb_bursts_clock_reset_and_gaps_never_divide_by_microsecond_interval():
    processor = AccelerationProcessor(calibration())
    outputs = []
    for i in range(100):
        for offset in (0.0, 0.00001):
            result = processor.push(i * 0.02 + offset, REST, 0.0)
            if result:
                outputs.append(result)
    assert len(outputs) > 30
    assert max(np.linalg.norm(a) for _, a in outputs) < 1e-12
    assert processor.push(0.0, REST, 0.0) is None
    assert processor.push(10.0, REST, 0.0) is None
    with pytest.raises(ValueError):
        processor.push(10.01, [np.nan, 0, 0], 0)
    assert processor.last is None


def _push_planar(processor, stamp, ax):
    raw = REST + ROTATION.T @ [ax, 0.0, 0.0]
    return processor.push(stamp, raw, 0.0)


def test_isolated_acceleration_spike_is_held_with_higher_covariance():
    processor = AccelerationProcessor(calibration(), period=0.01)
    for i in range(20):
        _push_planar(processor, i * 0.02, 0.0)

    # 一个串口样本异常；输出采用最新真实时间戳，但只发布高不确定度的旧估计。
    assert _push_planar(processor, 0.40, 4.0) is None
    stamp, acceleration = _push_planar(processor, 0.42, 0.0)
    assert stamp == pytest.approx(0.42)
    assert abs(acceleration[0]) < 0.05
    assert processor.last_covariance_scale == 25.0

    _push_planar(processor, 0.44, 0.0)
    stamp, acceleration = _push_planar(processor, 0.46, 0.0)
    assert stamp == pytest.approx(0.46)
    assert abs(acceleration[0]) < 0.05
    assert processor.last_covariance_scale == 1.0


def test_sustained_acceleration_is_accepted_after_short_confirmation():
    processor = AccelerationProcessor(calibration(), period=0.01)
    outputs = []
    for i in range(40):
        value = 2.0 if i >= 20 else 0.0
        result = _push_planar(processor, i * 0.02, value)
        if result is not None:
            outputs.append(result[1][0])
    assert outputs[-1] > 1.8
    assert max(outputs) > 1.5  # 不把真实持续超限运动无限钳在 INS 上限内


def test_fast_ramp_does_not_keep_refreshing_suspect_forever():
    processor = AccelerationProcessor(calibration(), period=0.01)
    outputs = []
    for i in range(40):
        value = max(0.0, i - 19) * 0.3
        result = _push_planar(processor, i * 0.02, value)
        if result is not None:
            outputs.append(result[1][0])
    assert outputs[-1] > 2.0


def test_long_input_gap_resets_history_without_fabricated_samples():
    processor = AccelerationProcessor(calibration(), period=0.01, max_gap=0.2)
    for i in range(20):
        _push_planar(processor, i * 0.02, 0.0)
    assert _push_planar(processor, 1.0, 1.0) is None
    assert processor.filtered is None
    assert processor.last_covariance_scale == 1.0
    assert _push_planar(processor, 1.02, 1.0) is None
    # 重新积累角速度历史后，第一帧采用新数据，不跨断流平滑或回填。
    first = None
    for i in range(2, 10):
        first = _push_planar(processor, 1.0 + i * 0.02, 1.0)
        if first is not None:
            break
    assert first is not None
    assert first[1][0] == pytest.approx(1.0)


def test_yaw_and_gravity_alone_cannot_calibrate_forward_axis():
    with pytest.raises(ValueError, match="激励不足"):
        fit_calibration(np.tile(REST, (2000, 1)), np.tile(REST, (30, 1)), np.zeros(30))
    with pytest.raises(ValueError, match="旋转矩阵"):
        PlanarCalibration(-np.eye(3), [0, 0, -9.81], [0.1] * 3)


def test_rotating_or_shocked_static_record_is_rejected():
    sample = np.c_[np.arange(2000) * 0.01, np.tile(REST, (2000, 1)), np.zeros(2000)]
    assert len(static_samples(sample)) == 2000
    sample[100, 4] = 0.4
    with pytest.raises(ValueError, match="旋转"):
        static_samples(sample)
